#include "own_contact.h"
#include "hand_broker.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

using namespace HiggsHands;
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
extern "C" int own_contact_c_probe(void);

struct Fixture {
    OwnContactService contacts;
    OwnContactFrame frame;
    uint64_t owner = 0, other = 0, now = Now();
    Fixture() {
        Service().ObserveWorld(600, false, now); Service().MarkMainThread();
        HIGGS_HandClientInfo c{}; c.structSize = sizeof(c); std::strcpy(c.name, "contact-service");
        CHECK(!Service().Register(c, owner)); std::strcpy(c.name, "other"); CHECK(!Service().Register(c, other));
        frame.epoch = Service().Epoch(); frame.observation = 1; frame.observedAt = now;
        frame.invalidationTicket = contacts.SourceTicket(); frame.count = 2;
        for (unsigned i = 0; i < 2; ++i) {
            auto& item = frame.items[i]; item.structSize = sizeof(item); item.physicalHand = 1u << i;
            item.source = HIGGS_CONTACT_ITEM_EQUIPPED; item.itemClass = HIGGS_CONTACT_CLASS_WEAPON;
            item.baseFormId = 100 + i; item.provenance = HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER;
            item.itemInstance = 1000 + i; item.itemGeneration = item.bodyGeneration = item.handGeneration = 1;
        }
        CHECK(contacts.Publish(frame, HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER, now));
    }
    ~Fixture() { CHECK(!Service().Unregister(owner)); CHECK(!Service().Unregister(other)); }
    HIGGS_CosmeticContactPolicy Policy() {
        HIGGS_CosmeticContactPolicy p{}; p.structSize = sizeof(p); p.epoch = frame.epoch;
        p.feedbackKinds = HIGGS_COSMETIC_SOUND | HIGGS_COSMETIC_HAPTIC;
        p.leftSources = p.rightSources = HIGGS_CONTACT_ITEM_EQUIPPED;
        p.leftClasses = p.rightClasses = HIGGS_CONTACT_CLASS_WEAPON;
        p.provenanceMask = HIGGS_CONTACT_PROVENANCE_EQUIPPED_COLLIDER;
        p.minRelativeSpeed = .1f; p.minClosingSpeed = .05f;
        p.cooldownMs = 10; p.leaseMs = 1000; p.maxObservationAgeMs = 100; return p;
    }
    HIGGS_CosmeticContactState Arm() {
        HIGGS_CosmeticContactState s{}; CHECK(!contacts.Request(owner, Policy(), s, now)); return s;
    }
    HIGGS_NativeOwnContact Input() {
        HIGGS_NativeOwnContact c{}; c.structSize = sizeof(c); c.producer = HIGGS_NATIVE_OWN_CONTACT_PLANCK;
        c.bodyA = 10; c.bodyB = 20; c.world = 1; c.mainFrame = c.physicsTick = 1;
        c.bToANormal[0] = -1; c.pointVelocityA[0] = 1;
        c.contactFlags = HIGGS_NATIVE_OWN_RESPONSE_DISABLED | HIGGS_NATIVE_OWN_HIT_PATH_SKIPPED; return c;
    }
    HIGGS_HandStatus Observe(const HIGGS_NativeOwnContact& c, uint64_t& sequence) {
        return contacts.Observe(frame, frame.items[0], frame.items[1], c, now, sequence);
    }
    uint32_t Read(uint64_t lease) {
        HIGGS_CosmeticContactQuery q{}; q.structSize = sizeof(q); q.epoch = frame.epoch; q.leaseId = lease;
        HIGGS_CosmeticContact value{}; value.structSize = sizeof(value); uint32_t count = 999;
        CHECK(!contacts.Contacts(owner, q, &value, 1, count, now)); return count;
    }
};

void NumericAndOwnership() {
    Fixture f; auto state = f.Arm(); auto input = f.Input(); uint64_t sequence = 0;
    CHECK(!f.Observe(input, sequence)); CHECK(sequence && f.Read(state.leaseId) == 1);
    HIGGS_CosmeticContactQuery q{}; q.structSize = sizeof(q); q.epoch = f.frame.epoch; q.leaseId = state.leaseId;
    uint32_t required = 99;
    CHECK(f.contacts.Contacts(f.owner, q, nullptr, 0, required, f.now) == HIGGS_HAND_BUFFER_TOO_SMALL);
    CHECK(required == 1);
    HIGGS_CosmeticContact value{}; value.structSize = sizeof(value);
    CHECK(!f.contacts.Contacts(f.owner, q, &value, 1, required, f.now));
    CHECK(value.relativeSpeed == 1 && value.closingSpeed == 1);
    CHECK(value.left.physicalHand == HIGGS_HAND_LEFT && value.right.physicalHand == HIGGS_HAND_RIGHT);
    CHECK((value.flags & 15u) == 15u); CHECK(value.policyRevision == state.revision);
    CHECK(f.contacts.Contacts(f.other, q, &value, 1, required, f.now) == HIGGS_HAND_WRONG_OWNER);
    CHECK(f.contacts.End(f.other, state.leaseId, state.epoch, f.now) == HIGGS_HAND_WRONG_OWNER);
    CHECK(f.Observe(input, sequence) == HIGGS_HAND_NOT_FOUND); // Same manifold/tick.
    input.physicsTick++; CHECK(f.Observe(input, sequence) == HIGGS_HAND_NOT_FOUND); // Cooldown.
    input.pointVelocityA[0] = input.pointVelocityB[0] = 100;
    CHECK(f.Observe(input, sequence) == HIGGS_HAND_NOT_FOUND); // Common motion has no relative impact.
    input = f.Input(); input.bToANormal[0] = 0;
    CHECK(f.Observe(input, sequence) == HIGGS_HAND_INVALID_ARGUMENT);
    input = f.Input(); input.pointVelocityA[0] = std::numeric_limits<float>::quiet_NaN();
    CHECK(f.Observe(input, sequence) == HIGGS_HAND_INVALID_ARGUMENT);
    CHECK(!f.contacts.End(f.owner, state.leaseId, state.epoch, f.now)); CHECK(!f.Read(state.leaseId));
}

void RevisionsFreshnessAndInvalidation() {
    Fixture f; const auto initial = f.Arm(); auto input = f.Input(); uint64_t sequence = 0;
    CHECK(!f.Observe(input, sequence));
    auto p = f.Policy(); p.leaseId = initial.leaseId; p.expectedRevision = initial.revision;
    HIGGS_CosmeticContactState next{}; CHECK(!f.contacts.Request(f.owner, p, next, f.now));
    CHECK(next.revision == initial.revision + 1); CHECK(!f.Read(initial.leaseId));
    CHECK(f.contacts.Request(f.owner, p, next, f.now) == HIGGS_HAND_CONFLICT);
    f.now += UINT64_C(11000000); input.physicsTick++;
    CHECK(!f.Observe(input, sequence)); CHECK(f.Read(initial.leaseId) == 1);
    f.now += UINT64_C(100000001); CHECK(!f.Read(initial.leaseId));
    input.physicsTick++; CHECK(f.Observe(input, sequence) == HIGGS_HAND_NOT_FOUND);
    const auto old = f.frame;
    f.contacts.Invalidate(); CHECK(f.Observe(input, sequence) == HIGGS_HAND_NOT_READY);
    CHECK(!f.contacts.Publish(old, HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER, f.now));
    f.frame.invalidationTicket = f.contacts.SourceTicket(); ++f.frame.observation; f.frame.observedAt = f.now;
    ++f.frame.items[0].bodyGeneration;
    CHECK(f.contacts.Publish(f.frame, HIGGS_OWN_CONTACT_CAP_EQUIPPED_COLLIDER, f.now));
    CHECK(!f.Read(initial.leaseId)); CHECK(!f.Observe(input, sequence));
    f.now = next.expiresAtNs;
    CHECK(!f.contacts.Copy(f.owner, initial.leaseId, next, f.now));
    CHECK(next.stage == HIGGS_COSMETIC_ENDED && next.reason == HIGGS_HAND_STALE);
    CHECK(!f.Read(initial.leaseId));
}

int main() {
    CHECK(!own_contact_c_probe()); NumericAndOwnership(); RevisionsFreshnessAndInvalidation();
    std::printf("PASS own-contact service %u checks (copied identities and contact inputs only)\n", checks);
}
