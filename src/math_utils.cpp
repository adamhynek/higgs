#include "math_utils.h"
#include "utils.h"
#include "config.h"
#include "finger_curves.h"

#include "skse64/NiGeometry.h"
#include "skse64/GameRTTI.h"

#include <array>
#include <unordered_set>


NiPoint3 CrossProduct(const NiPoint3 &vec1, const NiPoint3 &vec2)
{
    NiPoint3 result;
    result.x = vec1.y * vec2.z - vec1.z * vec2.y;
    result.y = vec1.z * vec2.x - vec1.x * vec2.z;
    result.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return result;
}

NiMatrix33 MatrixFromAxisAngle(const NiPoint3 &axis, float theta)
{
    NiPoint3 a = axis;
    float cosTheta = cosf(theta);
    float sinTheta = sinf(theta);
    NiMatrix33 result;

    result.data[0][0] = cosTheta + a.x*a.x*(1 - cosTheta);
    result.data[0][1] = a.x*a.y*(1 - cosTheta) - a.z*sinTheta;
    result.data[0][2] = a.x*a.z*(1 - cosTheta) + a.y*sinTheta;

    result.data[1][0] = a.y*a.x*(1 - cosTheta) + a.z*sinTheta;
    result.data[1][1] = cosTheta + a.y*a.y*(1 - cosTheta);
    result.data[1][2] = a.y*a.z*(1 - cosTheta) - a.x*sinTheta;

    result.data[2][0] = a.z*a.x*(1 - cosTheta) - a.y*sinTheta;
    result.data[2][1] = a.z*a.y*(1 - cosTheta) + a.x*sinTheta;
    result.data[2][2] = cosTheta + a.z*a.z*(1 - cosTheta);

    return result;
}

float RotationAngle(const NiMatrix33 &rot)
{
    float trace = rot.data[0][0] + rot.data[1][1] + rot.data[2][2];
    return acosf((trace - 1.0f) / 2.0f);
}

std::pair<NiPoint3, float> QuaternionToAxisAngle(const NiQuaternion &q)
{
    float angle = 2.0f * acosf(q.m_fW);
    float s = sqrtf(1.0f - q.m_fW * q.m_fW);

    NiPoint3 axis;
    if (s < 0.001f) {
        axis.x = 1.0f;
        axis.y = 0.0f;
        axis.z = 0.0f;
    }
    else {
        axis.x = q.m_fX / s;
        axis.y = q.m_fY / s;
        axis.z = q.m_fZ / s;
    }
    return { axis, angle };
}

NiPoint3 NiMatrixToYawPitchRoll(NiMatrix33 &mat)
{
    NiPoint3 euler;
    NiMatrixToYawPitchRollImpl(&mat, &euler.x, &euler.y, &euler.z);
    return euler;
}

NiPoint3 NiMatrixToEuler(NiMatrix33 &mat)
{
    NiPoint3 euler;
    NiMatrixToYawPitchRollImpl(&mat, &euler.z, &euler.x, &euler.y);
    return euler;
}

NiPoint3 MatrixToEuler(const NiMatrix33 &mat)
{
    // Thanks DavidJCobb
    NiPoint3 output(0, 0, 0);

    float fY = asin(((float)(SInt32)(-mat.arr[2] * 1000000)) / 1000000);
    float fCY = cos(fY);
    float fCYTest = ((float)(SInt32)(fCY * 100)) / 100;
    float fTX, fTY;
    if (fCY && abs(fCY) >= 0.00000011920929 && fCYTest) {
        fTX = mat.arr[8] / fCY;
        fTY = mat.arr[5] / fCY;
        output.x = atan2(fTY, fTX);
        fTX = mat.arr[0] / fCY;
        fTY = mat.arr[1] / fCY;
        output.z = atan2(fTY, fTX);
    }
    else {
        output.x = 0;
        fTX = mat.arr[4];
        fTY = mat.arr[3];
        output.z = -atan2(fTY, fTX);
    }
    output.y = fY;
    return output;
}

NiMatrix33 EulerToMatrix(const NiPoint3 &euler)
{
    // Thanks DavidJCobb
    NiMatrix33 output;
    //
    float _x = euler.x;
    float _y = euler.y;
    float _z = euler.z;

    float fSinX = sin(_x);
    float fSinY = sin(_y);
    float fSinZ = sin(_z);
    float fCosX = cos(_x);
    float fCosY = cos(_y);
    float fCosZ = cos(_z);
    //
    // Build the matrix.
    //
    output.data[0][0] = fCosY * fCosZ; // 1,1
    output.data[0][1] = fCosY * fSinZ;
    output.data[0][2] = -fSinY;
    output.data[1][0] = fSinX * fSinY * fCosZ - fCosX * fSinZ; // 2,1
    output.data[1][1] = fSinX * fSinY * fSinZ + fCosX * fCosZ;
    output.data[1][2] = fSinX * fCosY;
    output.data[2][0] = fCosX * fSinY * fCosZ + fSinX * fSinZ; // 3,1
    output.data[2][1] = fCosX * fSinY * fSinZ - fSinX * fCosZ;
    output.data[2][2] = fCosX * fCosY;
    //
    return output;
};

NiPoint3 NifskopeMatrixToEuler(const NiMatrix33 &in)
{
    const float(&m)[3][3] = in.data;
    NiPoint3 out;

    if (m[0][2] < 1.0) {
        if (m[0][2] > -1.0) {
            out.x = atan2(-m[1][2], m[2][2]);
            out.y = asin(m[0][2]);
            out.z = atan2(-m[0][1], m[0][0]);
        }
        else {
            out.x = -atan2(-m[1][0], m[1][1]);
            out.y = -M_PI / 2;
            out.z = 0.0;
        }
    }
    else {
        out.x = atan2(m[1][0], m[1][1]);
        out.y = M_PI / 2;
        out.z = 0.0;
    }
    return out;
}

NiMatrix33 NifskopeEulerToMatrix(const NiPoint3 &in)
{
    float sinX = sin(in.x);
    float cosX = cos(in.x);
    float sinY = sin(in.y);
    float cosY = cos(in.y);
    float sinZ = sin(in.z);
    float cosZ = cos(in.z);

    NiMatrix33 out;

    out.data[0][0] = cosY * cosZ;
    out.data[0][1] = -cosY * sinZ;
    out.data[0][2] = sinY;
    out.data[1][0] = sinX * sinY * cosZ + sinZ * cosX;
    out.data[1][1] = cosX * cosZ - sinX * sinY * sinZ;
    out.data[1][2] = -sinX * cosY;
    out.data[2][0] = sinX * sinZ - cosX * sinY * cosZ;
    out.data[2][1] = cosX * sinY * sinZ + sinX * cosZ;
    out.data[2][2] = cosX * cosY;

    return out;
}

NiMatrix33 MatrixFromForwardVector(NiPoint3 &forward, NiPoint3 &world)
{
    NiMatrix33 rot;
    NiMatrixFromForwardVector(&rot, &forward, &world);
    return rot;
}

NiPoint3 RotateVectorByAxisAngle(const NiPoint3 &vector, const NiPoint3 &axis, float angle)
{
    // Rodrigues' rotation formula
    float cosTheta = cosf(angle);
    return vector * cosTheta + (CrossProduct(axis, vector) * sinf(angle)) + axis * DotProduct(axis, vector) * (1.0f - cosTheta);
}

NiPoint3 RotateVectorByQuaternion(const NiQuaternion &quat, const NiPoint3 &vec)
{
    NiPoint3 qreal = { quat.m_fW, quat.m_fW, quat.m_fW };
    NiPoint3 q2minus1 = NiPoint3(qreal.x * qreal.x, qreal.y * qreal.y, qreal.z * qreal.z) - NiPoint3(0.5, 0.5, 0.5);

    NiPoint3 ret;
    //ret.setMul4(q2minus1, direction);
    ret = { q2minus1.x * vec.x, q2minus1.y * vec.y, q2minus1.z * vec.z };

    //hkReal imagDotDir = quat.getImag().dot3(direction);
    float imagDotDir = DotProduct({ quat.m_fX, quat.m_fY, quat.m_fZ }, vec);

    //ret.addMul4(imagDotDir, quat.getImag());
    ret += {quat.m_fX *imagDotDir, quat.m_fY *imagDotDir, quat.m_fZ *imagDotDir};

    NiPoint3 imagCrossDir;
    //imagCrossDir.setCross(quat.getImag(), direction);
    imagCrossDir = CrossProduct({ quat.m_fX, quat.m_fY, quat.m_fZ }, vec);
    //ret.addMul4(qreal, imagCrossDir);
    ret += {qreal.x *imagCrossDir.x, qreal.y *imagCrossDir.y, qreal.z *imagCrossDir.z};

    //this->setAdd4(ret, ret);
    return ret + ret;
}

NiPoint3 RotateVectorByInverseQuaternion(const NiQuaternion &quat, const NiPoint3 &vec)
{
    NiPoint3 qreal = { quat.m_fW, quat.m_fW, quat.m_fW };
    NiPoint3 q2minus1 = NiPoint3(qreal.x * qreal.x, qreal.y * qreal.y, qreal.z * qreal.z) - NiPoint3(0.5, 0.5, 0.5);

    NiPoint3 ret;
    //ret.setMul4(q2minus1, direction);
    ret = { q2minus1.x * vec.x, q2minus1.y * vec.y, q2minus1.z * vec.z };

    //hkReal imagDotDir = quat.getImag().dot3(direction);
    float imagDotDir = DotProduct({ quat.m_fX, quat.m_fY, quat.m_fZ }, vec);

    //ret.addMul4(imagDotDir, quat.getImag());
    ret += {quat.m_fX *imagDotDir, quat.m_fY *imagDotDir, quat.m_fZ *imagDotDir};

    NiPoint3 imagCrossDir;
    //imagCrossDir.setCross(quat.getImag(), direction);
    imagCrossDir = CrossProduct(vec, { quat.m_fX, quat.m_fY, quat.m_fZ });
    //ret.addMul4(qreal, imagCrossDir);
    ret += {qreal.x *imagCrossDir.x, qreal.y *imagCrossDir.y, qreal.z *imagCrossDir.z};

    //this->setAdd4(ret, ret);
    return ret + ret;
}

NiPoint3 ProjectVectorOntoPlane(const NiPoint3 &vector, const NiPoint3 &normal)
{
    NiPoint3 vectorAlongNormal = normal * DotProduct(vector, normal); // project above vector onto normal
    return vector - vectorAlongNormal;
}

NiTransform RotateTransformAboutPoint(NiTransform &transform, NiPoint3 &point, NiMatrix33 &rotation)
{
    NiTransform newTransform;
    newTransform.pos = point + rotation * (transform.pos - point);
    newTransform.rot = rotation * transform.rot;
    newTransform.scale = transform.scale;
    return newTransform;
}

std::pair<NiQuaternion, NiQuaternion> SwingTwistDecomposition(NiQuaternion &rotation, NiPoint3 &direction) {
    NiPoint3 rotationAxis = { rotation.m_fX, rotation.m_fY, rotation.m_fZ };
    float dot = DotProduct(direction, rotationAxis);
    NiPoint3 projection = direction * dot;
    NiQuaternion twist = { rotation.m_fW, projection.x, projection.y, projection.z };
    if (dot < 0.0f) {
        // Ensure twist points towards direction
        twist.m_fX *= -1;
        twist.m_fY *= -1;
        twist.m_fZ *= -1;
        twist.m_fW *= -1;
        // Rotation angle twist.angle() is now reliable
    }

    twist = QuaternionNormalized(twist);
    NiQuaternion swing = QuaternionMultiply(rotation, QuaternionInverse(twist));
    return { swing, twist };
}

void NiMatrixToHkMatrix(const NiMatrix33 &niMat, hkMatrix3 &hkMat)
{
    hkMat.setCols({ niMat.data[0][0], niMat.data[1][0], niMat.data[2][0], 0 },
        { niMat.data[0][1], niMat.data[1][1], niMat.data[2][1], 0 },
        { niMat.data[0][2], niMat.data[1][2], niMat.data[2][2], 0 });
}

hkRotation NiMatrixToHkMatrix(const NiMatrix33 &niMat)
{
    hkRotation out;
    NiMatrixToHkMatrix(niMat, out);
    return out;
}

void HkMatrixToNiMatrix(const hkMatrix3 &hkMat, NiMatrix33 &niMat)
{
    hkVector4 col0, col1, col2;
    hkMat.getCols(col0, col1, col2);

    niMat.data[0][0] = col0(0);
    niMat.data[1][0] = col0(1);
    niMat.data[2][0] = col0(2);

    niMat.data[0][1] = col1(0);
    niMat.data[1][1] = col1(1);
    niMat.data[2][1] = col1(2);

    niMat.data[0][2] = col2(0);
    niMat.data[1][2] = col2(1);
    niMat.data[2][2] = col2(2);
}

NiMatrix33 HkMatrixToNiMatrix(const hkMatrix3 &hkMat)
{
    NiMatrix33 out;
    HkMatrixToNiMatrix(hkMat, out);
    return out;
}

NiTransform hkTransformToNiTransform(const hkTransform &t, float scale, bool useHavokScale)
{
    NiTransform out;
    out.pos = HkVectorToNiPoint(t.m_translation) * (useHavokScale ? *g_inverseHavokWorldScale : 1.f);
    HkMatrixToNiMatrix(t.m_rotation, out.rot);
    out.scale = scale;
    return out;
}

hkTransform NiTransformTohkTransform(const NiTransform &t, bool useHavokScale)
{
    hkTransform out;
    out.m_translation = NiPointToHkVector(t.pos * (useHavokScale ? *g_havokWorldScale : 1.f));
    NiMatrixToHkMatrix(t.rot, out.m_rotation);
    return out;
}

NiMatrix33 QuaternionToMatrix(const NiQuaternion &q)
{
    double sqw = q.m_fW*q.m_fW;
    double sqx = q.m_fX*q.m_fX;
    double sqy = q.m_fY*q.m_fY;
    double sqz = q.m_fZ*q.m_fZ;

    NiMatrix33 m;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    m.data[0][0] = (sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    m.data[1][1] = (-sqx + sqy - sqz + sqw)*invs;
    m.data[2][2] = (-sqx - sqy + sqz + sqw)*invs;

    double tmp1 = q.m_fX*q.m_fY;
    double tmp2 = q.m_fZ*q.m_fW;
    m.data[1][0] = 2.0 * (tmp1 + tmp2)*invs;
    m.data[0][1] = 2.0 * (tmp1 - tmp2)*invs;

    tmp1 = q.m_fX*q.m_fZ;
    tmp2 = q.m_fY*q.m_fW;
    m.data[2][0] = 2.0 * (tmp1 - tmp2)*invs;
    m.data[0][2] = 2.0 * (tmp1 + tmp2)*invs;
    tmp1 = q.m_fY*q.m_fZ;
    tmp2 = q.m_fX*q.m_fW;
    m.data[2][1] = 2.0 * (tmp1 + tmp2)*invs;
    m.data[1][2] = 2.0 * (tmp1 - tmp2)*invs;

    return m;
}

NiQuaternion QuaternionIdentity()
{
    return { 1, 0, 0, 0 };
}

NiQuaternion QuaternionNormalized(const NiQuaternion &q)
{
    float length = QuaternionLength(q);
    if (length > 0.0f) {
        return QuaternionMultiply(q, 1.0f / length);
    }
    return QuaternionIdentity();
}

NiQuaternion QuaternionMultiply(const NiQuaternion &qa, const NiQuaternion &qb)
{
    NiQuaternion multiple;
    multiple.m_fW = qa.m_fW * qb.m_fW - qa.m_fX * qb.m_fX - qa.m_fY * qb.m_fY - qa.m_fZ * qb.m_fZ;
    multiple.m_fX = qa.m_fW * qb.m_fX + qa.m_fX * qb.m_fW + qa.m_fY * qb.m_fZ - qa.m_fZ * qb.m_fY;
    multiple.m_fY = qa.m_fW * qb.m_fY - qa.m_fX * qb.m_fZ + qa.m_fY * qb.m_fW + qa.m_fZ * qb.m_fX;
    multiple.m_fZ = qa.m_fW * qb.m_fZ + qa.m_fX * qb.m_fY - qa.m_fY * qb.m_fX + qa.m_fZ * qb.m_fW;
    return multiple;
}

NiQuaternion QuaternionMultiply(const NiQuaternion &q, float multiplier)
{
    NiQuaternion multiple;
    multiple.m_fW = q.m_fW * multiplier;
    multiple.m_fX = q.m_fX * multiplier;
    multiple.m_fY = q.m_fY * multiplier;
    multiple.m_fZ = q.m_fZ * multiplier;
    return multiple;
}

NiQuaternion QuaternionInverse(const NiQuaternion &q)
{
    NiQuaternion inverse;
    float normSquared = q.m_fW*q.m_fW + q.m_fX*q.m_fX + q.m_fY*q.m_fY + q.m_fZ*q.m_fZ;
    if (normSquared <= 0.0001)
        normSquared = 1;
    inverse.m_fW = q.m_fW / normSquared;
    inverse.m_fX = -q.m_fX / normSquared;
    inverse.m_fY = -q.m_fY / normSquared;
    inverse.m_fZ = -q.m_fZ / normSquared;
    return inverse;
}

NiQuaternion slerp(const NiQuaternion &qa, const NiQuaternion &qb, double t)
{
    // quaternion to return
    NiQuaternion qm;
    // Calculate angle between them.
    float cosHalfTheta = DotProduct(qa, qb);
    // if qa=qb or qa=-qb then theta = 0 and we can return qb
    if (fabs(cosHalfTheta) >= 0.99999) { // I actually experimentally determined this value. The value where I got this code was 0.9995 which is way too low for small angles
        qm.m_fW = qb.m_fW;
        qm.m_fX = qb.m_fX;
        qm.m_fY = qb.m_fY;
        qm.m_fZ = qb.m_fZ;
        return qm;
    }

    // If the dot product is negative, slerp won't take
    // the shorter path. Note that qb and -qb are equivalent when
    // the negation is applied to all four components. Fix by 
    // reversing one quaternion.
    NiQuaternion q2 = qb;
    if (cosHalfTheta < 0) {
        q2.m_fW *= -1;
        q2.m_fX *= -1;
        q2.m_fY *= -1;
        q2.m_fZ *= -1;
        cosHalfTheta *= -1;
    }

    // Calculate temporary values.
    float halfTheta = acosf(cosHalfTheta);
    float sinHalfTheta = sqrtf(1.0 - cosHalfTheta * cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // we could rotate around any axis normal to qa or qb
    if (fabs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
        qm.m_fW = (qa.m_fW * 0.5 + q2.m_fW * 0.5);
        qm.m_fX = (qa.m_fX * 0.5 + q2.m_fX * 0.5);
        qm.m_fY = (qa.m_fY * 0.5 + q2.m_fY * 0.5);
        qm.m_fZ = (qa.m_fZ * 0.5 + q2.m_fZ * 0.5);
        return qm;
    }
    float ratioA = sinf((1 - t) * halfTheta) / sinHalfTheta;
    float ratioB = sinf(t * halfTheta) / sinHalfTheta;
    // calculate Quaternion
    qm.m_fW = (qa.m_fW * ratioA + q2.m_fW * ratioB);
    qm.m_fX = (qa.m_fX * ratioA + q2.m_fX * ratioB);
    qm.m_fY = (qa.m_fY * ratioA + q2.m_fY * ratioB);
    qm.m_fZ = (qa.m_fZ * ratioA + q2.m_fZ * ratioB);
    return qm;
}

NiTransform lerp(NiTransform& a, NiTransform& b, double t)
{
    NiTransform lerped{};
    lerped.pos = lerp(a.pos, b.pos, t);
    lerped.scale = lerp(a.scale, b.scale, t);
    lerped.rot = QuaternionToMatrix(slerp(MatrixToQuaternion(a.rot), MatrixToQuaternion(b.rot), t));
    return lerped;
}

float AdvanceFloat(float current, float target, float speed)
{
    float delta = target - current;
    if (abs(delta) > speed * *g_deltaTime) {
        return current + speed * *g_deltaTime * (delta > 0 ? 1 : -1);
    }
    return target;
}

NiPoint3 AdvancePosition(const NiPoint3 &currentPos, const NiPoint3 &targetPos, float speed)
{
    NiPoint3 deltaDir = VectorNormalized(targetPos - currentPos);
    NiPoint3 deltaPos = deltaDir * speed * *g_deltaTime;
    if (VectorLengthSquared(deltaPos) < VectorLengthSquared(targetPos - currentPos)) {
        return currentPos + deltaPos;
    }
    return targetPos;
}

std::optional<NiTransform> AdvanceTransform(const NiTransform &currentTransform, const NiTransform &targetTransform, float posSpeed, float rotSpeed)
{
    NiQuaternion currentQuat = MatrixToQuaternion(currentTransform.rot);
    NiQuaternion desiredQuat = MatrixToQuaternion(targetTransform.rot);

    float deltaAngle = rotSpeed * 0.0174533f * *g_deltaTime;
    float quatAngle = QuaternionAngle(currentQuat, desiredQuat);

    NiPoint3 deltaDir = VectorNormalized(targetTransform.pos - currentTransform.pos);
    NiPoint3 deltaPos = deltaDir * posSpeed * *g_deltaTime;

    bool doRotation = deltaAngle < quatAngle;
    bool doTranslation = VectorLengthSquared(deltaPos) < VectorLengthSquared(targetTransform.pos - currentTransform.pos);

    if (doRotation || doTranslation) {
        // Rotation or position is not yet close enough

        NiTransform advancedTransform = targetTransform;
        if (doRotation) {
            // update rotation
            double slerpAmount = deltaAngle / quatAngle;
            NiQuaternion newQuat = slerp(currentQuat, desiredQuat, slerpAmount);
            newQuat = QuaternionNormalized(newQuat);
            advancedTransform.rot = QuaternionToMatrix(newQuat);
        }

        if (doTranslation) {
            // If not close enough, move the object closer to the hand at some velocity
            advancedTransform.pos = currentTransform.pos + deltaPos;
        }

        return advancedTransform;
    }
    return std::nullopt;
}

std::optional<NiTransform> AdvanceTransformSpeedMultiplied(const NiTransform &currentTransform, const NiTransform &targetTransform, float posSpeedMult, float rotSpeedMult)
{
    NiQuaternion currentQuat = MatrixToQuaternion(currentTransform.rot);
    NiQuaternion desiredQuat = MatrixToQuaternion(targetTransform.rot);

    float angleDiffAmount = QuaternionAngle(currentQuat, desiredQuat);
    float posDiffAmount = VectorLength(targetTransform.pos - currentTransform.pos);

    return AdvanceTransform(currentTransform, targetTransform, posDiffAmount * posSpeedMult, angleDiffAmount * rotSpeedMult);
}

float Determinant33(const NiMatrix33 &m)
{
    float a = m.data[0][0];
    float b = m.data[0][1];
    float c = m.data[0][2];
    float d = m.data[1][0];
    float e = m.data[1][1];
    float f = m.data[1][2];
    float g = m.data[2][0];
    float h = m.data[2][1];
    float i = m.data[2][2];
    return a * (e*i - f * h) - b * (d*i - f * g) + c * (d*h - e * g);
}

NiPoint3 QuadraticFromPoints(const NiPoint2 &p1, const NiPoint2 &p2, const NiPoint2 &p3)
{
    // Fit a quadratic to the 3 given points, and return the coefficients of x^2, x^1, x^0
    float x1sqr = p1.x*p1.x;
    float x2sqr = p2.x*p2.x;
    float x3sqr = p3.x*p3.x;

    float inverseDet = x1sqr * (p2.x - p3.x) - p1.x * (x2sqr - x3sqr) + (x2sqr*p3.x - x3sqr * p2.x);
    if (abs(inverseDet) <= 0.0001f) {
        return NiPoint3();
    }
    inverseDet = 1.0f / inverseDet;

    NiMatrix33 i; // inverse

    i.data[0][0] = p2.x - p3.x;
    i.data[0][1] = p3.x - p1.x;
    i.data[0][2] = p1.x - p2.x;

    i.data[1][0] = x3sqr - x2sqr;
    i.data[1][1] = x1sqr - x3sqr;
    i.data[1][2] = x2sqr - x1sqr;

    i.data[2][0] = x2sqr * p3.x - x3sqr * p2.x;
    i.data[2][1] = x3sqr * p1.x - x1sqr * p3.x;
    i.data[2][2] = x1sqr * p2.x - x2sqr * p1.x;

    i = i * inverseDet;

    NiPoint3 yVals = { p1.y, p2.y, p3.y };
    return i * yVals;
}

Point2::Point2()
{
    x = 0.0f;
    y = 0.0f;
}

Point2 Point2::operator- () const
{
    return Point2(-x, -y);
}

Point2 Point2::operator+ (const Point2& pt) const
{
    return Point2(x + pt.x, y + pt.y);
}

Point2 Point2::operator- (const Point2& pt) const
{
    return Point2(x - pt.x, y - pt.y);
}

Point2& Point2::operator+= (const Point2& pt)
{
    x += pt.x;
    y += pt.y;
    return *this;
}
Point2& Point2::operator-= (const Point2& pt)
{
    x -= pt.x;
    y -= pt.y;
    return *this;
}

// Scalar operations
Point2 Point2::operator* (float scalar) const
{
    return Point2(scalar * x, scalar * y);
}
Point2 Point2::operator/ (float scalar) const
{
    float invScalar = 1.0f / scalar;
    return Point2(invScalar * x, invScalar * y);
}

Point2& Point2::operator*= (float scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}
Point2& Point2::operator/= (float scalar)
{
    float invScalar = 1.0f / scalar;
    x *= invScalar;
    y *= invScalar;
    return *this;
}


namespace MathUtils
{
    Result GetClosestPointOnTriangle(const NiPoint3 &point, const TriangleData &triangle)
    {
        // Taken from https://www.geometrictools.com/GTE//Mathematics/DistPointTriangleExact.h and adapted

        NiPoint3 pos0 = triangle.v0;
        NiPoint3 pos1 = triangle.v1;
        NiPoint3 pos2 = triangle.v2;

        NiPoint3 diff = point - pos0;
        NiPoint3 edge0 = pos1 - pos0;
        NiPoint3 edge1 = pos2 - pos0;
        float a00 = DotProduct(edge0, edge0);
        float a01 = DotProduct(edge0, edge1);
        float a11 = DotProduct(edge1, edge1);
        float b0 = -DotProduct(diff, edge0);
        float b1 = -DotProduct(diff, edge1);
        float det = a00 * a11 - a01 * a01;
        float t0 = a01 * b1 - a11 * b0;
        float t1 = a01 * b0 - a00 * b1;

        if (t0 + t1 <= det)
        {
            if (t0 < 0)
            {
                if (t1 < 0)  // region 4
                {
                    if (b0 < 0)
                    {
                        t1 = 0;
                        if (-b0 >= a00)  // V1
                        {
                            t0 = 1;
                        }
                        else  // E01
                        {
                            t0 = -b0 / a00;
                        }
                    }
                    else
                    {
                        t0 = 0;
                        if (b1 >= 0)  // V0
                        {
                            t1 = 0;
                        }
                        else if (-b1 >= a11)  // V2
                        {
                            t1 = 1;
                        }
                        else  // E20
                        {
                            t1 = -b1 / a11;
                        }
                    }
                }
                else  // region 3
                {
                    t0 = 0;
                    if (b1 >= 0)  // V0
                    {
                        t1 = 0;
                    }
                    else if (-b1 >= a11)  // V2
                    {
                        t1 = 1;
                    }
                    else  // E20
                    {
                        t1 = -b1 / a11;
                    }
                }
            }
            else if (t1 < 0)  // region 5
            {
                t1 = 0;
                if (b0 >= 0)  // V0
                {
                    t0 = 0;
                }
                else if (-b0 >= a00)  // V1
                {
                    t0 = 1;
                }
                else  // E01
                {
                    t0 = -b0 / a00;
                }
            }
            else  // region 0, interior
            {
                float invDet = 1 / det;
                t0 *= invDet;
                t1 *= invDet;
            }
        }
        else
        {
            float tmp0, tmp1, numer, denom;

            if (t0 < 0)  // region 2
            {
                tmp0 = a01 + b0;
                tmp1 = a11 + b1;
                if (tmp1 > tmp0)
                {
                    numer = tmp1 - tmp0;
                    denom = a00 - (float)2 * a01 + a11;
                    if (numer >= denom)  // V1
                    {
                        t0 = 1;
                        t1 = 0;
                    }
                    else  // E12
                    {
                        t0 = numer / denom;
                        t1 = 1 - t0;
                    }
                }
                else
                {
                    t0 = 0;
                    if (tmp1 <= 0)  // V2
                    {
                        t1 = 1;
                    }
                    else if (b1 >= 0)  // V0
                    {
                        t1 = 0;
                    }
                    else  // E20
                    {
                        t1 = -b1 / a11;
                    }
                }
            }
            else if (t1 < 0)  // region 6
            {
                tmp0 = a01 + b1;
                tmp1 = a00 + b0;
                if (tmp1 > tmp0)
                {
                    numer = tmp1 - tmp0;
                    denom = a00 - (float)2 * a01 + a11;
                    if (numer >= denom)  // V2
                    {
                        t1 = 1;
                        t0 = 0;
                    }
                    else  // E12
                    {
                        t1 = numer / denom;
                        t0 = 1 - t1;
                    }
                }
                else
                {
                    t1 = 0;
                    if (tmp1 <= 0)  // V1
                    {
                        t0 = 1;
                    }
                    else if (b0 >= 0)  // V0
                    {
                        t0 = 0;
                    }
                    else  // E01
                    {
                        t0 = -b0 / a00;
                    }
                }
            }
            else  // region 1
            {
                numer = a11 + b1 - a01 - b0;
                if (numer <= 0)  // V2
                {
                    t0 = 0;
                    t1 = 1;
                }
                else
                {
                    denom = a00 - (float)2 * a01 + a11;
                    if (numer >= denom)  // V1
                    {
                        t0 = 1;
                        t1 = 0;
                    }
                    else  // 12
                    {
                        t0 = numer / denom;
                        t1 = 1 - t0;
                    }
                }
            }
        }

        Result result;
        result.parameter[0] = 1 - t0 - t1;
        result.parameter[1] = t0;
        result.parameter[2] = t1;
        result.closest = pos0 + (edge0 * t0) + (edge1 * t1);
        diff = point - result.closest;
        result.sqrDistance = DotProduct(diff, diff);
        return result;
    }

    bool RayIntersectsTriangle(const NiPoint3 &rayOrigin,
        const NiPoint3 &rayVector,
        const Triangle &triangle,
        NiPoint3 &outIntersectionPoint,
        uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
    {
        const float EPSILON = 0.0000001;

        uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
        NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
        vert = (vertices + triangle.vertexIndices[1] * vertexStride);
        NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
        vert = (vertices + triangle.vertexIndices[2] * vertexStride);
        NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

        NiPoint3 edge1, edge2, h, s, q;
        float a, f, u, v;
        edge1 = vertex1 - vertex0;
        edge2 = vertex2 - vertex0;
        h = CrossProduct(rayVector, edge2);
        a = DotProduct(edge1, h);
        if (a > -EPSILON && a < EPSILON)
            return false;    // This ray is parallel to this triangle.
        f = 1.0 / a;
        s = rayOrigin - vertex0;
        u = f * DotProduct(s, h);
        if (u < 0.0 || u > 1.0)
            return false;
        q = CrossProduct(s, edge1);
        v = f * DotProduct(rayVector, q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = f * DotProduct(edge2, q);
        if (t > EPSILON) // ray intersection
        {
            outIntersectionPoint = rayOrigin + rayVector * t;
            return true;
        }
        else // This means that there is a line intersection but not a ray intersection.
            return false;
    }

    bool GetClosestPointOnTriangleToLine(const NiPoint3 &rayOrigin,
        const NiPoint3 &rayVector,
        const TriangleData &triangle,
        NiPoint3 &outIntersectionPoint,
        float &outSqrDistance,
        bool &outIntersects)
    {
        const float EPSILON = 0.0000001;

        NiPoint3 vertex0 = triangle.v0;
        NiPoint3 vertex1 = triangle.v1;
        NiPoint3 vertex2 = triangle.v2;

        bool intersects = true;
        NiPoint3 edge1, edge2, h, s, q;
        float a, f, u, v;
        edge1 = vertex1 - vertex0;
        edge2 = vertex2 - vertex0;
        h = CrossProduct(rayVector, edge2);
        a = DotProduct(edge1, h);
        if (a > -EPSILON && a < EPSILON)
            return false;    // This ray is parallel to this triangle.
        f = 1.0 / a;
        s = rayOrigin - vertex0;
        u = f * DotProduct(s, h);
        if (u < 0.0 || u > 1.0)
            intersects = false;
        q = CrossProduct(s, edge1);
        v = f * DotProduct(rayVector, q);
        if (v < 0.0 || u + v > 1.0)
            intersects = false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = f * DotProduct(edge2, q);
        //if (t > EPSILON) // ray intersection
        //{

        NiPoint3 intersectPoint = rayOrigin + rayVector * t;
        Result result = GetClosestPointOnTriangle(intersectPoint, triangle);
        outIntersectionPoint = result.closest;
        outSqrDistance = result.sqrDistance;
        outIntersects = intersects;
        return true;
        //}
        //else // This means that there is a line intersection but not a ray intersection.
        //	return false;
    }

    NiPoint3 GetClosestPointOnLineSegment(const NiPoint3 &start, const NiPoint3 &end, const NiPoint3 &point)
    {
        float l2 = VectorLengthSquared(end - start);  // i.e. |w-v|^2
        if (l2 == 0.0) return start;   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line. 
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        // We clamp t from [0,1] to handle points outside the segment vw.
        float t = max(0, min(1, DotProduct(point - start, end - start) / l2));
        NiPoint3 projection = start + (end - start) * t;  // Projection falls on the segment
        return projection;
    }

    Point2 GetClosestPointOnLineSegment(const Point2 &start, const Point2 &end, const Point2 &point)
    {
        float l2 = VectorLengthSquared(end - start);
        if (l2 == 0.0) return start;
        float t = max(0, min(1, DotProduct(point - start, end - start) / l2));
        Point2 projection = start + (end - start) * t;
        return projection;
    }

    NiPoint3 GetFurthestPointOnLineSegment(const NiPoint3 &start, const NiPoint3 &end, const NiPoint3 &point)
    {
        float l2 = VectorLengthSquared(end - start);
        if (l2 == 0.0) return start;   // v == w case

        return VectorLengthSquared(start - point) >= VectorLengthSquared(end - point) ? start : end;
    }

    float LineSegmentLineSegmentDistance(const Point2 &a, const Point2 &b, const Point2 &c, const Point2 &d)
    {
        return min(
            VectorLength(GetClosestPointOnLineSegment(c, d, a) - a),
            min(
                VectorLength(GetClosestPointOnLineSegment(c, d, b) - b),
                min(
                    VectorLength(GetClosestPointOnLineSegment(a, b, c) - c),
                    VectorLength(GetClosestPointOnLineSegment(a, b, d) - d)
                )
            )
        );
    }

    // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
    // intersect the intersection point may be stored in the floats i_x and i_y.
    bool LineSegmentIntersectsLineSegment(const Point2 &p0, const Point2 &p1, const Point2 &p2, const Point2 &p3, Point2 *intersection)
    {
        Point2 s1 = p1 - p0;
        Point2 s2 = p3 - p2;

        float det = (-s2.x * s1.y + s1.x * s2.y);
        if (fabsf(det) < 0.0000001)
            return false; // line segments are parallel

        float s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / det;
        float t = (s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / det;

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            if (intersection) {
                *intersection = p0 + s1 * t;
            }
            return true;
        }

        return false; // No collision
    }

    bool LinePlaneIntersection(NiPoint3 &contact, const NiPoint3 &ray, const NiPoint3 &rayOrigin,
        const NiPoint3 &normal, const NiPoint3 &coord)
    {
        // get d value
        float d = DotProduct(normal, coord);

        float normalDotRay = DotProduct(normal, ray);
        if (normalDotRay < 0.0000001) {
            return false; // No intersection, the line is parallel to the plane
        }

        // Compute the X value for the directed line ray intersecting the plane
        float x = (d - DotProduct(normal, rayOrigin)) / normalDotRay;

        // output contact point
        contact = rayOrigin + ray * x; //Make sure your ray vector is normalized
        return true;
    }

    bool PlaneIntersectsLineSegment(const NiPoint3 &planePoint, const NiPoint3 &planeNormal, const NiPoint3 &segmentStart, const NiPoint3 &segmentFinish, NiPoint3 &outPoint)
    {
        NiPoint3 edge = segmentFinish - segmentStart;
        float edgeLength = VectorLength(edge);
        NiPoint3 ray = edgeLength ? edge / edgeLength : NiPoint3();

        // get d value, where plane eq: n*x = d
        float d = DotProduct(planeNormal, planePoint);

        float normalDotRay = DotProduct(planeNormal, ray);
        if (abs(normalDotRay) < 0.0000001) {
            return false; // No intersection, the line is parallel to the plane
        }

        // Compute the X value for the directed line ray intersecting the plane
        float x = (d - DotProduct(planeNormal, segmentStart)) / normalDotRay;

        // output contact point
        if (x < 0 || x > edgeLength) {
            // intersection point is not on the line segment
            return false;
        }

        outPoint = segmentStart + ray * x;
        return true;
    }

    // Return # of intersection points (0, 1, or 2)
    int CircleIntersectsTriangle(const NiPoint3 &circleCenter,
        const NiPoint3 &circleNormal,
        float circleRadius,
        const TriangleData &triangle,
        NiPoint3 &outIntersectionPoint1,
        NiPoint3 &outIntersectionPoint2)
    {
        NiPoint3 vertex0 = triangle.v0;
        NiPoint3 vertex1 = triangle.v1;
        NiPoint3 vertex2 = triangle.v2;

        // Check each triangle edge for intersection. Either none intersect, or 2 of them do.
        NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
        bool edge1Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex0, vertex1, edge1Intersection);
        bool edge2Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex0, vertex2, edge2Intersection);
        if (!edge1Intersects && !edge2Intersects) {
            return 0; // Impossible for 2 edges to intersect at this point
        }
        bool edge3Intersects = PlaneIntersectsLineSegment(circleCenter, circleNormal, vertex1, vertex2, edge3Intersection);

        int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
        if (numIntersections < 2) {
            return 0;
        }

        // p: start point of intersection line; d: direction vector of intersection line
        NiPoint3 d, p;
        float edgeLength;
        if (edge1Intersects && edge2Intersects) {
            NiPoint3 edge = edge2Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - circleCenter; // Make the circle center the origin, it makes the math cleaner
        }
        else if (edge1Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - circleCenter;
        }
        else if (edge2Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge2Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge2Intersection - circleCenter;
        }
        else {
            // Impossible
            ASSERT(false);
            return 0;
        }

        // Solve quadratic for pt that is on the intersection line as well as on the circle (dist to circle center == radius)
        float pDotd = DotProduct(p, d);
        float dLength2 = VectorLengthSquared(d);
        float pLength2 = VectorLengthSquared(p);
        float discriminant = pDotd * pDotd - dLength2 * (pLength2 - circleRadius * circleRadius);
        if (discriminant < 0) {
            return 0;
        }
        float t1 = (-pDotd + sqrtf(discriminant)) / dLength2;
        float t2 = (-pDotd - sqrtf(discriminant)) / dLength2;

        bool t1IsOnSegment = t1 >= 0 && t1 <= edgeLength;
        bool t2IsOnSegment = t2 >= 0 && t2 <= edgeLength;

        if (t1IsOnSegment && t2IsOnSegment) {
            // This is rare... the circle intersects the triangle at 2 points
            outIntersectionPoint1 = circleCenter + p + d * t1; // Add back the circle center after making it the origin
            outIntersectionPoint2 = circleCenter + p + d * t2;
            return 2;
        }
        else if (t1IsOnSegment) {
            // t1 is on the segment
            outIntersectionPoint1 = circleCenter + p + d * t1;
            return 1;
        }
        else if (t2IsOnSegment) {
            // t2 is on the segment
            outIntersectionPoint1 = circleCenter + p + d * t2;
            return 1;
        }

        return 0;
    }

    // Return # of intersection points (0, 1, or 2)
    int DiskIntersectsTriangle(const NiPoint3 &diskCenter,
        const NiPoint3 &diskNormal,
        float diskRadius,
        const Triangle &triangle,
        NiPoint3 &outIntersectionPoint1,
        NiPoint3 &outIntersectionPoint2,
        uintptr_t vertices, UInt8 vertexStride, UInt32 vertexPosOffset)
    {
        uintptr_t vert = (vertices + triangle.vertexIndices[0] * vertexStride);
        NiPoint3 vertex0 = *(NiPoint3 *)(vert + vertexPosOffset);
        vert = (vertices + triangle.vertexIndices[1] * vertexStride);
        NiPoint3 vertex1 = *(NiPoint3 *)(vert + vertexPosOffset);
        vert = (vertices + triangle.vertexIndices[2] * vertexStride);
        NiPoint3 vertex2 = *(NiPoint3 *)(vert + vertexPosOffset);

        // Check each triangle edge for intersection. Either none intersect, or 2 of them do.
        NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
        bool edge1Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex1, edge1Intersection);
        bool edge2Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex0, vertex2, edge2Intersection);
        if (!edge1Intersects && !edge2Intersects) {
            return 0; // Impossible for 2 edges to intersect at this point
        }
        bool edge3Intersects = PlaneIntersectsLineSegment(diskCenter, diskNormal, vertex1, vertex2, edge3Intersection);

        int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
        if (numIntersections < 2) {
            return 0;
        }

        // p: start point of intersection line; d: direction vector of intersection line
        NiPoint3 d, p;
        float edgeLength;
        if (edge1Intersects && edge2Intersects) {
            NiPoint3 edge = edge2Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - diskCenter; // Make the circle center the origin, it makes the math cleaner
        }
        else if (edge1Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - diskCenter;
        }
        else if (edge2Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge2Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge2Intersection - diskCenter;
        }
        else {
            // Impossible
            return 0;
        }

        NiPoint3 furthestPoint = GetFurthestPointOnLineSegment(p, p + d * edgeLength, { 0, 0, 0 }); // circle center is the origin, so {0, 0, 0}
        if (VectorLength(furthestPoint) > diskRadius) {
            // Furthest point is past the end of the disk. See if there is another point at the edge of the disk.

            // Solve quadratic for pt that is on the intersection line as well as on the circle (dist to circle center == radius)
            float pDotd = DotProduct(p, d);
            float dLength2 = VectorLengthSquared(d);
            float pLength2 = VectorLengthSquared(p);
            float discriminant = pDotd * pDotd - dLength2 * (pLength2 - diskRadius * diskRadius);
            if (discriminant < 0) {
                return 0;
            }
            float t1 = (-pDotd + sqrtf(discriminant)) / dLength2;
            float t2 = (-pDotd - sqrtf(discriminant)) / dLength2;

            bool t1IsOnSegment = t1 >= 0 && t1 <= edgeLength;
            bool t2IsOnSegment = t2 >= 0 && t2 <= edgeLength;

            if (t1IsOnSegment && t2IsOnSegment) {
                // This is rare... the circle intersects the triangle at 2 points
                outIntersectionPoint1 = diskCenter + p + d * t1; // Add back the circle center after making it the origin
                outIntersectionPoint2 = diskCenter + p + d * t2;
                return 2;
            }
            else if (t1IsOnSegment) {
                // t1 is on the segment
                outIntersectionPoint1 = diskCenter + p + d * t1;
                return 1;
            }
            else if (t2IsOnSegment) {
                // t2 is on the segment
                outIntersectionPoint1 = diskCenter + p + d * t2;
                return 1;
            }
        }
        else {
            // Intersection line is entirely inside the disk
            outIntersectionPoint1 = diskCenter + furthestPoint;
            return 1;
        }

        return 0;
    }

    // Return whether tip, outer, inner intersect
    std::tuple<bool, bool, bool> FingerIntersectsTriangle(int fingerIndex,
        const NiPoint3 &fingerCenter,
        const NiPoint3 &fingerNormal,
        const NiPoint3 &zeroAngleVector,
        float handScale,
        float nodeScale,
        const TriangleData &triangle,
        float &outTipAngle,
        float &outOuterAngle,
        float &outInnerAngle)
    {
        NiPoint3 vertex0 = triangle.v0;
        NiPoint3 vertex1 = triangle.v1;
        NiPoint3 vertex2 = triangle.v2;

        // Check each triangle edge for intersection. Either none intersect, or 2 of them do.
        NiPoint3 edge1Intersection, edge2Intersection, edge3Intersection;
        bool edge1Intersects = PlaneIntersectsLineSegment(fingerCenter, fingerNormal, vertex0, vertex1, edge1Intersection);
        bool edge2Intersects = PlaneIntersectsLineSegment(fingerCenter, fingerNormal, vertex0, vertex2, edge2Intersection);
        if (!edge1Intersects && !edge2Intersects) {
            return { false, false, false }; // Impossible for 2 edges to intersect at this point
        }
        bool edge3Intersects = PlaneIntersectsLineSegment(fingerCenter, fingerNormal, vertex1, vertex2, edge3Intersection);

        int numIntersections = edge1Intersects + edge2Intersects + edge3Intersects;
        if (numIntersections < 2) {
            return { false, false, false };
        }

        // p: start point of intersection line; d: direction vector of intersection line
        NiPoint3 d, p;
        float edgeLength;
        if (edge1Intersects && edge2Intersects) {
            NiPoint3 edge = edge2Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - fingerCenter; // Make the circle center the origin, it makes the math cleaner
        }
        else if (edge1Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge1Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge1Intersection - fingerCenter;
        }
        else if (edge2Intersects && edge3Intersects) {
            NiPoint3 edge = edge3Intersection - edge2Intersection;
            edgeLength = VectorLength(edge);
            d = edgeLength ? edge / edgeLength : NiPoint3();
            p = edge2Intersection - fingerCenter;
        }
        else {
            // Impossible
            return { false, false, false };
        }

        NiPoint3 edgeStart = p;
        NiPoint3 edgeEnd = p + d * edgeLength;


        float startAngle = acosf(DotProduct(VectorNormalized(edgeStart), zeroAngleVector));
        if (DotProduct(fingerNormal, CrossProduct(zeroAngleVector, edgeStart)) < 0) {
            // Positive angles are those which curl the finger
            startAngle *= -1;
        }

        float endAngle = acosf(DotProduct(VectorNormalized(edgeEnd), zeroAngleVector));
        if (DotProduct(fingerNormal, CrossProduct(zeroAngleVector, edgeEnd)) < 0) {
            endAngle *= -1;
        }

        float scale = handScale / nodeScale;

        auto CurveCheck = [=, &fingerNormal, &zeroAngleVector, &vertex0, &vertex1, &vertex2]
        (SavedFingerData fingerVals[], float startAngle, float endAngle, float &outAngle) -> bool
        {
            bool crossesBehind = false;
            float smallestAngle = min(startAngle, endAngle);
            float minAllowedAngle = fingerVals[0].angle + g_minAllowedFingerAngle;

            auto CircleIntersection = [&]() -> bool
            {
                NiPoint3 circleIntersection1, circleIntersection2;
                int numCircleIntersections = CircleIntersectsTriangle(fingerCenter, fingerNormal, fingerVals[0].fingerLength * scale, triangle, circleIntersection1, circleIntersection2);
                if (numCircleIntersections > 0) {
                    NiPoint3 triNormal = VectorNormalized(CrossProduct(vertex1 - vertex0, vertex2 - vertex1));
                    if (numCircleIntersections > 1) {
                        NiPoint3 centerToIntersect1 = circleIntersection1 - fingerCenter;
                        NiPoint3 centerToIntersect2 = circleIntersection2 - fingerCenter;

                        float angle1 = acosf(DotProduct(VectorNormalized(centerToIntersect1), zeroAngleVector));
                        float angle2 = acosf(DotProduct(VectorNormalized(centerToIntersect2), zeroAngleVector));

                        NiPoint3 tangent1 = CrossProduct(fingerNormal, centerToIntersect1);
                        NiPoint3 tangent2 = CrossProduct(fingerNormal, centerToIntersect2);

                        bool angle1Smaller = angle1 <= angle2;
                        float smallerAngle = angle1Smaller ? angle1 : angle2;
                        float largerAngle = angle1Smaller ? angle2 : angle1;
                        NiPoint3 smallerAngleTangent = angle1Smaller ? tangent1 : tangent2;
                        NiPoint3 largerAngleTangent = angle1Smaller ? tangent2 : tangent1;
                        NiPoint3 smallerAngleCenterToIntersect = angle1Smaller ? centerToIntersect1 : centerToIntersect2;
                        NiPoint3 largerAngleCenterToIntersect = angle1Smaller ? centerToIntersect2 : centerToIntersect1;

                        if (DotProduct(triNormal, smallerAngleTangent) <= 0) {
                            float angle = smallerAngle;
                            if (DotProduct(fingerNormal, CrossProduct(zeroAngleVector, smallerAngleCenterToIntersect)) < 0) {
                                // Positive angles are those which curl the finger
                                angle *= -1;
                            }

                            if (angle > 0) {
                                // If the circle intersected at a positive angle but the finger missed, we don't care.
                                // We only do the circle intersection to catch points that are behind the finger...
                                return false;
                            }

                            _DMESSAGE("pt: %.2f", angle);
                            outAngle = angle;
                            return true;
                        }
                        else if (DotProduct(triNormal, largerAngleTangent) <= 0) {
                            float angle = -largerAngle;
                            _DMESSAGE("pt: %.2f", angle);
                            outAngle = angle;
                            return true;
                        }
                    }
                    else {
                        NiPoint3 centerToIntersect = circleIntersection1 - fingerCenter;
                        NiPoint3 tangent = CrossProduct(fingerNormal, centerToIntersect);

                        if (DotProduct(triNormal, tangent) <= 0) {
                            float angle = acosf(DotProduct(VectorNormalized(centerToIntersect), zeroAngleVector));
                            if (DotProduct(fingerNormal, CrossProduct(zeroAngleVector, centerToIntersect)) < 0) {
                                // Positive angles are those which curl the finger
                                angle *= -1;
                            }

                            if (angle > 0) {
                                // If the circle intersected at a positive angle but the finger missed, we don't care.
                                // We only do the circle intersection to catch points that are behind the finger...
                                return false;
                            }

                            _DMESSAGE("pt: %.2f", angle);
                            outAngle = angle; // Something negative
                            return true;
                        }
                    }
                }
                return false;
            };

            if (max(startAngle, endAngle) < minAllowedAngle) {
                // Both ends of the edge are behind the finger. Derotate.

                return CircleIntersection();
            }
            else {
                if (smallestAngle < minAllowedAngle) {
                    // If one end of the edge is behind the finger and the other isn't, then derotate only if the finger missed in the normal case.
                    crossesBehind = true;
                }
            }

            bool startSmaller = startAngle < endAngle;
            float smallerAngle = startSmaller ? startAngle : endAngle;
            float largerAngle = startSmaller ? endAngle : startAngle;

            float lengthAtSmallerAngle = startSmaller ? VectorLength(edgeStart) : VectorLength(edgeEnd);
            float lengthAtLargerAngle = startSmaller ? VectorLength(edgeEnd) : VectorLength(edgeStart);

            Point2 smallerAnglePt = Point2(cosf(smallerAngle), sinf(smallerAngle)) * lengthAtSmallerAngle;
            Point2 largerAnglePt = Point2(cosf(largerAngle), sinf(largerAngle)) * lengthAtLargerAngle;

            // TODO: Revisit the crossesBehind thing and negative angles?
            float radiusAtSmallerAngle = -1.0f, radiusAtLargerAngle = -1.0f;
            SavedFingerData fingerData;
            int smallerIndex = LookupFingerByAngle(fingerVals, smallerAngle, &fingerData);
            radiusAtSmallerAngle = fingerData.fingerLength;
            radiusAtSmallerAngle *= scale;

            int largerIndex = LookupFingerByAngle(fingerVals, largerAngle, &fingerData);
            radiusAtLargerAngle = fingerData.fingerLength;
            radiusAtLargerAngle *= scale;

            if (lengthAtSmallerAngle > radiusAtSmallerAngle || lengthAtLargerAngle > radiusAtLargerAngle) {
                // Edge is not entirely within the curve area. Now we only care if it intersects the curve.

                //_MESSAGE("%.3f\t%.2f %.2f", VectorLength(smallerPt - largerPt), smallerAngle * 57.2958f, largerAngle * 57.2958f);

                int end = min(largerIndex, g_numFingerVals - 2);
                for (int i = smallerIndex; i <= end; i++) {
                    float angle = fingerVals[i].angle;
                    float length = fingerVals[i].fingerLength;
                    length *= scale;

                    float nextAngle = fingerVals[i + 1].angle;
                    float nextLength = fingerVals[i + 1].fingerLength;
                    nextLength *= scale;

                    Point2 currentPt = Point2(cosf(angle), sinf(angle)) * length;
                    Point2 nextPt = Point2(cosf(nextAngle), sinf(nextAngle)) * nextLength;

                    Point2 intersection;
                    //_MESSAGE("%.3f\t%.2f %.2f", LineSegmentLineSegmentDistance(smallerPt, largerPt, currentPt, nextPt), angle, angleCorrection);
                    if (LineSegmentIntersectsLineSegment(smallerAnglePt, largerAnglePt, currentPt, nextPt, &intersection) ||
                        LineSegmentLineSegmentDistance(smallerAnglePt, largerAnglePt, currentPt, nextPt) <= 0.015f) {

                        NiPoint3 triNormal = VectorNormalized(CrossProduct(vertex1 - vertex0, vertex2 - vertex1));

                        NiPoint3 currentPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, fingerNormal, angle)) * length;
                        NiPoint3 nextPtDiskspace = VectorNormalized(RotateVectorByAxisAngle(zeroAngleVector, fingerNormal, nextAngle)) * nextLength;
                        NiPoint3 tangent = nextPtDiskspace - currentPtDiskspace;

                        if (DotProduct(triNormal, tangent) <= 0) {
                            // Front face of the triangle was intersected CCW around the circle
                            // TODO: Use precise intersection point instead of the current angle / length
                            _DMESSAGE("pt: %.2f", angle);
                            outAngle = angle;
                            return true;
                        }
                    }
                }
            }
            else {
                // Edge is entirely 'under' the curve
            }

            // If one end of the edge is behind the finger and the other isn't, then derotate only if the finger missed in the normal case.
            if (crossesBehind) {
                return CircleIntersection();
            }

            return false;
        };

        float tipAngle, outerAngle, innerAngle;
        bool tipIntersects = CurveCheck(g_fingerTipVals[fingerIndex], startAngle, endAngle, tipAngle);
        bool outerIntersects = CurveCheck(g_fingerOuterVals[fingerIndex], startAngle, endAngle, outerAngle);
        bool innerIntersects = CurveCheck(g_fingerInnerVals[fingerIndex], startAngle, endAngle, innerAngle);

        if (tipIntersects || outerIntersects || innerIntersects) {
            _DMESSAGE("inner outer tip: %d %d %d", innerIntersects, outerIntersects, tipIntersects);
        }

        outTipAngle = tipAngle;
        outOuterAngle = outerAngle;
        outInnerAngle = innerAngle;
        return { tipIntersects, outerIntersects, innerIntersects };
    }
}

bool IsHairGeometry(BSGeometry *geom)
{
    NiPointer<NiProperty> geomProperty = geom->m_spEffectState;
    if (geomProperty) {
        BSShaderProperty *shaderProperty = DYNAMIC_CAST(geomProperty, NiProperty, BSShaderProperty);
        if (shaderProperty) {
            BSShaderMaterialBase *material = shaderProperty->material;
            if (material) {
                if (material->GetShaderType() == BSShaderMaterial::kShaderType_HairTint) {
                    return true;
                }
            }
        }
    }
    return false;
}

struct BSDismemberSkinInstance_Data
{
    bool editorVisible; // 0
    bool startNetBoneSet; // 1
    UInt16 slot; // 2
};

bool IsHairSkinInstance(NiSkinInstance *skinInstance)
{
    BSDismemberSkinInstance *dismemberSkinInstance = DYNAMIC_CAST(skinInstance, NiSkinInstance, BSDismemberSkinInstance);
    if (!dismemberSkinInstance) return false;

    BSDismemberSkinInstance_Data *data = (BSDismemberSkinInstance_Data *)dismemberSkinInstance->partitionFlags;
    if (!data) return false;

    for (int i = 0; i < dismemberSkinInstance->numPartitions; i++) {
        if (data[i].slot != 31 && data[i].slot != 41) { // 31 is the hair bodypart number, 41 is long hair
            return false;
        }
    }

    return true;
}

bool IsIgnorableGeometry(BSGeometry *geom)
{
    NiPointer<NiProperty> geomProperty = geom->m_spEffectState;
    if (geomProperty) {
        BSShaderProperty *shaderProperty = DYNAMIC_CAST(geomProperty, NiProperty, BSShaderProperty);
        if (shaderProperty) {
            if (
                (Config::options.grabIgnoreDecal && (shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Decal || shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Dynamic_Decal)) ||
                (Config::options.grabIgnoreBlood && shaderProperty->shaderFlags2 & BSShaderProperty::ShaderFlags2::kSLSF2_Weapon_Blood) ||
                (Config::options.grabIgnoreSoftEffect && shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Soft_Effect)
            ) {
                return true;
            }
        }
    }
    return false;
}

bool ShouldIgnoreBasedOnVertexAlpha(BSTriShape *geom)
{
    NiPointer<NiProperty> geomProperty = geom->m_spEffectState;
    if (!geomProperty) return false;

    BSShaderProperty *shaderProperty = DYNAMIC_CAST(geomProperty, NiProperty, BSShaderProperty);
    if (!shaderProperty) return false;

    if (!(shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Vertex_Alpha)) return false;

    BSGeometryData *geomData = geom->geometryData;
    if (!geomData) return false;

    UInt64 vertexDesc = geom->vertexDesc;
    VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
    if (!(vertexFlags & VertexFlags::VF_COLORS)) return false;

    uintptr_t verts = (uintptr_t)(geomData->vertices);
    UInt16 numVerts = geom->numVertices;
    UInt8 vertexSize = (vertexDesc & 0xF) * 4;
    UInt32 colorOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_COLOR);

    if (verts && numVerts > 0) {
        float totalAlpha = 0.f;
        for (int i = 0; i < numVerts; i++) {
            uintptr_t vert = (verts + i * vertexSize);
            UInt32 color = *(UInt32 *)(vert + colorOffset);
            UInt8 alpha0to255 = (color >> 24) & 0xff;
            float alpha = float(alpha0to255) / 255.f;
            totalAlpha += alpha;
        }
        float avgAlpha = totalAlpha / numVerts;
        _MESSAGE("%s: %.3f avg alpha", geom->m_name, avgAlpha);
        if (avgAlpha < Config::options.geometryVertexAlphaThreshold) {
            return true;
        }
    }

    return false;
}

bool HasAlreadyProcessedSimilarSkinInstance(SkinInstanceRepresentation &query, std::vector<SkinInstanceRepresentation> &visitedSkinInstances)
{
    for (SkinInstanceRepresentation &skinInstance : visitedSkinInstances) {
        if (skinInstance.skinData == query.skinData && skinInstance.skinPartition == query.skinPartition && skinInstance.numBones == query.numBones) {
            bool allTransformsMatch = true;
            for (UInt32 i = 0; i < query.numBones; i++) {
                if (skinInstance.boneTransforms[i] != query.boneTransforms[i]) {
                    allTransformsMatch = false;
                    break;
                }
            }
            if (allTransformsMatch) {
                return true;
            }
        }
    }
    return false;
}

// Add triangles to the given list for each skinned partition in geom
void UpdateSkinnedTriangles(BSTriShape *geom, std::vector<TriangleData> &triangles, std::vector<TrianglePartitionData> &trianglePartitions, std::unordered_map<NiSkinPartition::Partition *, PartitionData> &partitionData, std::vector<SkinInstanceRepresentation> &visitedSkinInstances, std::unordered_set<NiAVObject *> *nodesToSkinTo = nullptr)
{
    if (geom->m_name && Config::options.grabNodeNameBlacklist.find(std::string_view(geom->m_name)) != Config::options.grabNodeNameBlacklist.end()) return;

    NiSkinInstancePtr skinInstance = geom->m_spSkinInstance;
    if (!geom->m_spSkinInstance) return;

    NiAVObject *skeletonRoot = skinInstance->m_pkRootParent;
    if (!skeletonRoot) return;

    NiSkinDataPtr skinData = skinInstance->m_spSkinData;
    if (!skinData) return;

    if (IsIgnorableGeometry(geom)) return;

    if (Config::options.disableGrabHair && (IsHairGeometry(geom) || IsHairSkinInstance(skinInstance))) {
        return;
    }

    NiSkinPartitionPtr skinPartition = skinInstance->m_spSkinPartition;
    if (!skinPartition) {
        skinPartition = skinData->m_spSkinPartition;
    }	

    bool hasPartitions = skinPartition && skinPartition->m_pkPartitions && skinPartition->m_uiPartitions > 0;
    if (!hasPartitions) {
        _MESSAGE("Skindata with no partitions");
        return;
    }

    UInt32 numBones = skinInstance->m_uiBoneNodes;
    NiTransform **boneTransforms = skinInstance->m_worldTransforms;
    if (!boneTransforms || numBones <= 0) return;

    BSDynamicTriShape *dynamicShape = DYNAMIC_CAST(geom, BSTriShape, BSDynamicTriShape);

    if (!dynamicShape) {
        SkinInstanceRepresentation skinInstanceRep = { skinData, skinPartition, boneTransforms, numBones };
        if (HasAlreadyProcessedSimilarSkinInstance(skinInstanceRep, visitedSkinInstances)) {
            return;
        }
        visitedSkinInstances.push_back(skinInstanceRep);
    }

    NiSkinData::BoneData *boneData = skinData->m_pkBoneData;

    UInt32 numTotalVerts = skinPartition->vertexCount;

    for (int i = 0; i < skinPartition->m_uiPartitions; i++) {
        NiSkinPartition::Partition &partition = skinPartition->m_pkPartitions[i];
        UInt16 *partBones = partition.m_pusBones;

        // Build up bone transforms
        UInt16 numPartBones = partition.m_usBones;
        std::vector<NiTransform> boneTrans(numPartBones);
        for (int t = 0; t < numPartBones; t++) {
            UInt16 boneIndex = partBones[t];
            if (boneIndex >= numBones) break;
            // bonePalette == Bone Indices. Store bone indices per vertex.
            // bones == Bones. Map from partition bones to skinInstance bones.

            NiTransform *boneTransform = boneTransforms[boneIndex];
            if (boneTransform) {
                NiSkinData::BoneData &data = boneData[boneIndex];
                boneTrans[t] = *boneTransform * data.m_kSkinToBone;
            }
        }

        // Compute vertex positions using bone transforms

        UInt16 numPartVerts = partition.m_usVertices;
        UInt16 numTris = partition.m_usTriangles;
        NiSkinPartition::TriShape *shapeData = partition.shapeData;
        if (!shapeData) continue;

        uintptr_t verts = (uintptr_t)shapeData->m_RawVertexData;
        auto tris = (Triangle *)shapeData->m_RawIndexData;

        UInt64 vertexDesc = shapeData->m_VertexDesc;
        VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
        UInt8 vertexSize = (vertexDesc & 0xF) * 4;
        UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

        if (dynamicShape) {
            verts = (uintptr_t)dynamicShape->pDynamicData;
            vertexSize = 16;
            posOffset = 0;
        }

        if ((vertexFlags & VertexFlags::VF_VERTEX || dynamicShape) && verts && numPartVerts > 0 && tris && numTris > 0) {
            UInt16 numWeightsPerVertex = partition.m_usBonesPerVertex;

            std::vector<NiPoint3> transVerts(numTotalVerts); // Allocate space for _all vertices for all partitions_ but only fill in the ones mapped to by the individual partition

            if (!partitionData.count(&partition)) {
                partitionData.insert({ &partition, PartitionData(skinInstance, verts) });
                std::vector<UInt16> &inverseVertMap = partitionData[&partition].globalVertToPartVertMap;
                inverseVertMap.assign(numTotalVerts, -1);
                for (int v = 0; v < numPartVerts; v++) {
                    // Reverse the mapping to map partition.shapeData->m_RawVertexData vertex to partition vertex
                    UInt16 vindex = partition.m_pusVertexMap[v];
                    if (vindex >= numTotalVerts) break;
                    inverseVertMap[vindex] = v;
                }
                partitionData[&partition].verticesWS.resize(numPartVerts);
            }

            std::unordered_set<UInt16> includeVerts;

            for (int v = 0; v < numPartVerts; v++) {
                // partition.m_pusVertexMap: maps from partition vertex -> partition.shapeData->m_RawVertexData vertex
                UInt16 vindex = partition.m_pusVertexMap[v];
                if (vindex >= numTotalVerts) break;

                uintptr_t vert = (verts + vindex * vertexSize);
                NiPoint3 vertPos = *(NiPoint3 *)(vert + posOffset);

                for (int w = 0; w < numWeightsPerVertex; w++) {
                    int offset = v * numWeightsPerVertex + w;
                    float weight = partition.m_pfWeights[offset];

                    if (weight > 0.0f) {
                        UInt16 partBoneIndex = partition.m_pucBonePalette[offset];

                        if (!nodesToSkinTo) {
                            includeVerts.insert(vindex);
                        }
                        else {
                            UInt16 skinBoneIndex = partBones[partBoneIndex];
                            NiAVObject *bone = skinInstance->m_ppkBones[skinBoneIndex];
                            if (bone && nodesToSkinTo->count(bone) != 0) {
                                includeVerts.insert(vindex);
                            }
                        }

                        NiTransform boneTransform = boneTrans[partBoneIndex];

                        transVerts[vindex] += boneTransform * vertPos * weight;
                    }
                }

                partitionData[&partition].verticesWS[v] = transVerts[vindex];
            }

            for (int t = 0; t < numTris; t++) {
                Triangle tri = tris[t];

                if (includeVerts.count(tri.vertexIndices[0]) != 0 || includeVerts.count(tri.vertexIndices[1]) != 0 || includeVerts.count(tri.vertexIndices[2]) != 0) {
                    TriangleData triData(tri, transVerts);
                    triangles.push_back(triData);
                    trianglePartitions.push_back({ partition, tri });
                }
            }

            _MESSAGE("%d skinned tris", numTris);
        }
    }
}

// Get skinned triangles for all geometry rooted at root
void GetSkinnedTriangles(NiAVObject *root, std::vector<TriangleData> &triangles, std::vector<TrianglePartitionData> &trianglePartitions, std::unordered_map<NiSkinPartition::Partition *, PartitionData> &partitionData, std::vector<SkinInstanceRepresentation> &visitedSkinInstances, std::unordered_set<NiAVObject *> *nodesToSkinTo)
{
    if (root->m_flags & 1) return; // Node is culled

    BSTriShape *geom = root->GetAsBSTriShape();
    if (geom) {
        UpdateSkinnedTriangles(geom, triangles, trianglePartitions, partitionData, visitedSkinInstances, nodesToSkinTo);
        return;
    }

    NiNode *node = root->GetAsNiNode();
    if (node) {
        if (node->GetAsNiSwitchNode()) {
            // NiSwitchNode: Return the first valid child
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    GetSkinnedTriangles(child, triangles, trianglePartitions, partitionData, visitedSkinInstances, nodesToSkinTo);
                    return;
                }
            }
        }
        else {
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    GetSkinnedTriangles(child, triangles, trianglePartitions, partitionData, visitedSkinInstances, nodesToSkinTo);
                }
            }
        }
    }
}

void UpdateTriangles(BSTriShape *geom, std::vector<TriangleData> &triangles, std::vector<NiAVObject *> &triangleNodes)
{
    if (geom->m_name && Config::options.grabNodeNameBlacklist.find(std::string_view(geom->m_name)) != Config::options.grabNodeNameBlacklist.end()) return;

    UInt16 numTris = geom->unk198;
    UInt16 numVerts = geom->numVertices;
    BSGeometryData *geomData = geom->geometryData;
    if (!geomData) {
        // Probably skinned mesh
        return;
    }

    if (IsIgnorableGeometry(geom)) return;

    if (Config::options.disableGrabGeometryWithVertexAlpha && ShouldIgnoreBasedOnVertexAlpha(geom)) return;

    _MESSAGE("%s: %d tris", geom->m_name, numTris);

    auto tris = (Triangle *)geomData->triangles;
    uintptr_t verts = (uintptr_t)(geomData->vertices);

    UInt64 vertexDesc = geom->vertexDesc;
    VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
    UInt8 vertexSize = (vertexDesc & 0xF) * 4;
    UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

    BSDynamicTriShape *dynamicShape = DYNAMIC_CAST(geom, BSTriShape, BSDynamicTriShape);
    if (dynamicShape) {
        verts = (uintptr_t)dynamicShape->pDynamicData;
        vertexSize = 16;
        posOffset = 0;
    }

    if ((vertexFlags & VertexFlags::VF_VERTEX) && verts && numVerts > 0 && tris && numTris > 0) {
        NiTransform &nodeTransform = geom->m_worldTransform;

        for (int i = 0; i < numTris; i++) {
            Triangle tri = tris[i];
            TriangleData triData(tri, verts, posOffset, vertexSize);
            triData.ApplyTransform(nodeTransform);
            triangles.push_back(triData);
            triangleNodes.push_back(geom);
        }
    }
}

void GetTriangles(NiAVObject *root, std::vector<TriangleData> &triangles, std::vector<NiAVObject *> &triangleNodes)
{
    if (root->m_flags & 1) return; // Node is culled

    BSTriShape *geom = root->GetAsBSTriShape();
    if (geom) {
        UpdateTriangles(geom, triangles, triangleNodes);
        return;
    }

    NiNode *node = root->GetAsNiNode();
    if (node) {
        if (node->GetAsNiSwitchNode()) {
            // NiSwitchNode: Return the first valid child
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    GetTriangles(child, triangles, triangleNodes);
                    return;
                }
            }
        }
        else {
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    GetTriangles(child, triangles, triangleNodes);
                }
            }
        }
    }
}

void GetFingerIntersectionOnGraphicsGeometry(std::vector<Intersection> &tipIntersections, std::vector<Intersection> &outerIntersections, std::vector<Intersection> &innerIntersections,
    const std::vector<TriangleData> &triangles,
    int fingerIndex, float handScale, const NiPoint3 &center, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector)
{
    for (int i = 0; i < triangles.size(); i++) {
        const TriangleData &triangle = triangles[i];

        // get closest point on triangle to given point
        float tipAngle, outerAngle, innerAngle;
        auto[tipIntersects, outerIntersects, innerIntersects] = MathUtils::FingerIntersectsTriangle(fingerIndex, center, normal, zeroAngleVector, handScale, 1.0f, triangle,
            tipAngle, outerAngle, innerAngle);

        if (tipIntersects) {
            tipIntersections.push_back({ tipAngle, i });
        }
        if (outerIntersects) {
            outerIntersections.push_back({ outerAngle, i });
        }
        if (innerIntersects) {
            innerIntersections.push_back({ innerAngle, i });
        }
    }
}

NiPoint3 GetClosestPointOnIntersection(const NiPoint3 &point, const OldIntersection &intersection)
{
    BSTriShape *geom = intersection.node;
    Triangle tri = intersection.tri;

    BSGeometryData *geomData = geom->geometryData;

    uintptr_t verts = (uintptr_t)(geomData->vertices);

    UInt64 vertexDesc = geom->vertexDesc;
    UInt8 vertexSize = (vertexDesc & 0xF) * 4;

    UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

    NiTransform nodeTransform = geom->m_worldTransform;

    TriangleData triData(tri, verts, posOffset, vertexSize);
    MathUtils::Result result = MathUtils::GetClosestPointOnTriangle(point, triData);

    return nodeTransform * result.closest;
}

std::array<NiPoint3, 3> GetVertices(const OldIntersection &intersection)
{
    BSTriShape *geom = intersection.node;
    Triangle tri = intersection.tri;

    BSGeometryData *geomData = geom->geometryData;

    uintptr_t verts = (uintptr_t)(geomData->vertices);

    UInt64 vertexDesc = geom->vertexDesc;
    UInt8 vertexSize = (vertexDesc & 0xF) * 4;

    UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

    NiTransform nodeTransform = geom->m_worldTransform;

    // get vertices in world space
    uintptr_t vert = (verts + tri.vertexIndices[0] * vertexSize);
    NiPoint3 pos0 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
    vert = (verts + tri.vertexIndices[1] * vertexSize);
    NiPoint3 pos1 = nodeTransform * *(NiPoint3 *)(vert + posOffset);
    vert = (verts + tri.vertexIndices[2] * vertexSize);
    NiPoint3 pos2 = nodeTransform * *(NiPoint3 *)(vert + posOffset);

    return { pos0, pos1, pos2 };
}

bool DoesAnyVertexMatch(const std::array<NiPoint3, 3> &verts1, const std::array<NiPoint3, 3> &verts2)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (VectorLengthSquared(verts1[i] - verts2[j]) < 0.00001f) {
                return true;
            }
        }
    }
    return false;
}


bool GetIntersections(const std::vector<TriangleData> &triangles, int fingerIndex, float handScale, const NiPoint3 &center, const NiPoint3 &normal, const NiPoint3 &zeroAngleVector,
    Intersection &outIntersection)
{
    std::vector<Intersection> tipIntersections, outerIntersections, innerIntersections;

    // Populate intersections
    GetFingerIntersectionOnGraphicsGeometry(tipIntersections, outerIntersections, innerIntersections, triangles, fingerIndex, handScale, center, normal, zeroAngleVector);

    auto visit = [](const Intersection &intersection, Intersection &smallestIntersection)
    {
        float angle = intersection.angle;
        float degrees = angle * 57.2958f;

        _DMESSAGE("angle: %.2f", degrees);

        if (angle >= 0 && angle < smallestIntersection.angle) {
            smallestIntersection = { angle, intersection.triangleIndex };
        }
    };

    auto CheckNegativeWithinAngle = [](const std::vector<Intersection> &intersections, float angle) -> SInt32
    {
        // Return the triangle index of an intersection with a negative intersection angle whose magnitude is within the given angle, or -1.

        for (auto &intersection : intersections) {
            float intersectionAngle = intersection.angle;
            if (intersectionAngle < 0 && abs(intersectionAngle) < angle) {
                return intersection.triangleIndex;
            }
        }
        return -1;
    };

    // Get best intersection angles for each curve
    Intersection innerIntersection{ (std::numeric_limits<float>::max)(), 0 };
    for (Intersection &intersection : innerIntersections) {
        visit(intersection, innerIntersection);
    }
    if (SInt32 innerNegativeIndex = CheckNegativeWithinAngle(innerIntersections, innerIntersection.angle); innerNegativeIndex >= 0) {
        outIntersection = { -1.f, innerNegativeIndex };
        return true;
    }

    Intersection outerIntersection{ (std::numeric_limits<float>::max)(), 0 };
    for (Intersection &intersection : outerIntersections) {
        visit(intersection, outerIntersection);
    }
    if (SInt32 outerNegativeIndex = CheckNegativeWithinAngle(outerIntersections, outerIntersection.angle); outerNegativeIndex >= 0) {
        outIntersection = { -1.f, outerNegativeIndex };
        return true;
    }

    Intersection tipIntersection{ (std::numeric_limits<float>::max)(), 0 };
    for (Intersection &intersection : tipIntersections) {
        visit(intersection, tipIntersection);
    }
    if (SInt32 tipNegativeIndex = CheckNegativeWithinAngle(tipIntersections, tipIntersection.angle);  tipNegativeIndex >= 0) {
        outIntersection = { -1.f, tipNegativeIndex };
        return true;
    }

    // Lookup curve vals from angles for each curve
    float innerVal = -1, outerVal = -1, tipVal = -1;
    SavedFingerData fingerData;
    if (innerIntersection.angle != (std::numeric_limits<float>::max)()) {
        LookupFingerByAngle(g_fingerInnerVals[fingerIndex], innerIntersection.angle, &fingerData);
        innerVal = fingerData.curveVal;
    }
    if (outerIntersection.angle != (std::numeric_limits<float>::max)()) {
        LookupFingerByAngle(g_fingerOuterVals[fingerIndex], outerIntersection.angle, &fingerData);
        outerVal = fingerData.curveVal;
    }
    if (tipIntersection.angle != (std::numeric_limits<float>::max)()) {
        LookupFingerByAngle(g_fingerTipVals[fingerIndex], tipIntersection.angle, &fingerData);
        tipVal = fingerData.curveVal;
    }

    // Pick the largest curve value (the one which curls the finger the least)
    float curveVal = max(innerVal, max(outerVal, tipVal));
    if (curveVal == -1) {
        return false;
    }

    if (curveVal == tipVal) {
        outIntersection = { curveVal, tipIntersection.triangleIndex };
    }
    else if (curveVal == outerVal) {
        outIntersection = { curveVal, outerIntersection.triangleIndex };
    }
    else { // curveVal == innerVal
        outIntersection = { curveVal, innerIntersection.triangleIndex };
    }

    return true;
}


bool GetClosestPointOnGraphicsGeometry(NiAVObject *root, const NiPoint3 &point, NiPoint3 *closestPos, NiPoint3 *closestNormal, float *closestDistanceSoFar)
{
    BSTriShape *geom = root->GetAsBSTriShape();
    if (geom) {
        UInt16 numTris = geom->unk198;
        UInt16 numVerts = geom->numVertices;
        BSGeometryData *geomData = geom->geometryData;
        if (!geomData) {
            // Probably skinned mesh
            return false;
        }

        _MESSAGE("%d tris", numTris);

        auto tris = (Triangle *)geomData->triangles;
        uintptr_t verts = (uintptr_t)(geomData->vertices);

        UInt64 vertexDesc = geom->vertexDesc;
        VertexFlags vertexFlags = NiSkinPartition::GetVertexFlags(vertexDesc);
        UInt8 vertexSize = (vertexDesc & 0xF) * 4;

        if ((vertexFlags & VertexFlags::VF_VERTEX) && verts && numVerts > 0 && tris && numTris > 0) {
            UInt32 posOffset = NiSkinPartition::GetVertexAttributeOffset(vertexDesc, VertexAttribute::VA_POSITION);

            NiTransform inverseTransform = InverseTransform(geom->m_worldTransform);
            NiPoint3 pointInNodeSpace = inverseTransform * point;

            int closestTri = -1;
            NiPoint3 closestTriPos;
            float closestDistance = *closestDistanceSoFar;

            for (int i = 0; i < numTris; i++) {
                Triangle tri = tris[i];
                TriangleData triData(tri, verts, posOffset, vertexSize);

                // get closest point on triangle to given point
                MathUtils::Result result = MathUtils::GetClosestPointOnTriangle(pointInNodeSpace, triData);
                float distance = result.sqrDistance;
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTriPos = result.closest;
                    closestTri = i;
                }
            }

            if (closestTri != -1) {
                *closestDistanceSoFar = closestDistance;
                *closestPos = geom->m_worldTransform * closestTriPos;

                Triangle closestTriangle = tris[closestTri];

                TriangleData triData(closestTriangle, verts, posOffset, vertexSize);
                triData.v0 = geom->m_worldTransform * triData.v0;
                triData.v1 = geom->m_worldTransform * triData.v1;
                triData.v2 = geom->m_worldTransform * triData.v2;

                *closestNormal = VectorNormalized(CrossProduct(triData.v1 - triData.v0, triData.v2 - triData.v1));

                return true;
            }
        }
        return false;
    }
    NiNode *node = root->GetAsNiNode();
    if (node) {
        if (node->GetAsNiSwitchNode()) {
            // NiSwitchNode: Return the first valid child
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    return GetClosestPointOnGraphicsGeometry(child, point, closestPos, closestNormal, closestDistanceSoFar);
                }
            }
        }
        else {
            bool success = false;
            for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
                auto child = node->m_children.m_data[i];
                if (child) {
                    if (GetClosestPointOnGraphicsGeometry(child, point, closestPos, closestNormal, closestDistanceSoFar)) {
                        success = true;
                    }
                }
            }
            return success;
        }
    }
    return false;
}

bool GetClosestPointOnGraphicsGeometryToLine(const std::vector<TriangleData> &tris, const NiPoint3 &point, const NiPoint3 &direction,
    NiPoint3 &closestPos, NiPoint3 &closestNormal, int &closestIndex, float &closestDistanceSoFar)
{
    int closestTri = -1;
    NiPoint3 closestTriPos;
    float closestDistance = closestDistanceSoFar;

    float lateralWeight = Config::options.grabLateralWeight;
    float directionalWeight = Config::options.grabDirectionalWeight;

    for (int i = 0; i < tris.size(); i++) {
        const TriangleData &triangle = tris[i];
        // get closest point on the triangle to given ray starting at point in direction
        NiPoint3 closestPoint;
        float closestDistSquared;
        bool intersects;
        MathUtils::GetClosestPointOnTriangleToLine(point, direction, triangle, closestPoint, closestDistSquared, intersects);

        NiPoint3 pointToClosest = closestPoint - point;
        NiPoint3 pointToClosestAlongDirection = direction * DotProduct(pointToClosest, direction);

        float directionalDistance = VectorLength(pointToClosestAlongDirection);
        float lateralDistance = VectorLength(pointToClosest - pointToClosestAlongDirection);
        float distance = directionalWeight * directionalDistance*directionalDistance + lateralWeight * lateralDistance*lateralDistance;
        if (distance < closestDistance) {
            NiPoint3 triNormal = VectorNormalized(CrossProduct(triangle.v1 - triangle.v0, triangle.v2 - triangle.v1));

            if (DotProduct(triNormal, direction) <= 0) {
                // Front face of the triangle faces the line
                closestDistance = distance;
                closestTriPos = closestPoint;
                closestTri = i;
            }
        }
    }

    if (closestTri != -1) {
        closestIndex = closestTri;
        closestDistanceSoFar = closestDistance;
        closestPos = closestTriPos;

        const TriangleData &triData = tris[closestTri];
        closestNormal = VectorNormalized(CrossProduct(triData.v1 - triData.v0, triData.v2 - triData.v1));

        return true;
    }

    return false;
}



namespace NiMathDouble
{
    NiPoint3::NiPoint3()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    NiPoint3::NiPoint3(const ::NiPoint3 &pointSingle)
    {
        x = pointSingle.x;
        y = pointSingle.y;
        z = pointSingle.z;
    }

    NiPoint3 NiPoint3::operator- () const
    {
        return NiPoint3(-x, -y, -z);
    }

    NiPoint3 NiPoint3::operator+ (const NiPoint3 &pt) const
    {
        return NiPoint3(x + pt.x, y + pt.y, z + pt.z);
    }

    NiPoint3 NiPoint3::operator- (const NiPoint3 &pt) const
    {
        return NiPoint3(x - pt.x, y - pt.y, z - pt.z);
    }

    NiPoint3 &NiPoint3::operator+= (const NiPoint3 &pt)
    {
        x += pt.x;
        y += pt.y;
        z += pt.z;
        return *this;
    }
    NiPoint3 &NiPoint3::operator-= (const NiPoint3 &pt)
    {
        x -= pt.x;
        y -= pt.y;
        z -= pt.z;
        return *this;
    }

    // Scalar operations
    NiPoint3 NiPoint3::operator* (double scalar) const
    {
        return NiPoint3(scalar * x, scalar * y, scalar * z);
    }
    NiPoint3 NiPoint3::operator/ (double scalar) const
    {
        double invScalar = 1.0 / scalar;
        return NiPoint3(invScalar * x, invScalar * y, invScalar * z);
    }

    NiPoint3 &NiPoint3::operator*= (double scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    NiPoint3 &NiPoint3::operator/= (double scalar)
    {
        double invScalar = 1.0 / scalar;
        x *= invScalar;
        y *= invScalar;
        z *= invScalar;
        return *this;
    }

    NiMatrix33::NiMatrix33(const ::NiMatrix33 &matSingle)
    {
        data[0][0] = matSingle.data[0][0];
        data[0][1] = matSingle.data[0][1];
        data[0][2] = matSingle.data[0][2];
        data[1][0] = matSingle.data[1][0];
        data[1][1] = matSingle.data[1][1];
        data[1][2] = matSingle.data[1][2];
        data[2][0] = matSingle.data[2][0];
        data[2][1] = matSingle.data[2][1];
        data[2][2] = matSingle.data[2][2];
    }

    ::NiMatrix33 NiMatrix33::ToSingle() const
    {
        ::NiMatrix33 matSingle;
        matSingle.data[0][0] = data[0][0];
        matSingle.data[0][1] = data[0][1];
        matSingle.data[0][2] = data[0][2];
        matSingle.data[1][0] = data[1][0];
        matSingle.data[1][1] = data[1][1];
        matSingle.data[1][2] = data[1][2];
        matSingle.data[2][0] = data[2][0];
        matSingle.data[2][1] = data[2][1];
        matSingle.data[2][2] = data[2][2];
        return matSingle;
    }

    void NiMatrix33::Identity()
    {
        data[0][0] = 1.0;
        data[0][1] = 0.0;
        data[0][2] = 0.0;
        data[1][0] = 0.0;
        data[1][1] = 1.0;
        data[1][2] = 0.0;
        data[2][0] = 0.0;
        data[2][1] = 0.0;
        data[2][2] = 1.0;
    }

    NiMatrix33 NiMatrix33::operator* (const NiMatrix33 &rhs) const
    {
        NiMatrix33 tmp;
        tmp.data[0][0] =
            data[0][0] * rhs.data[0][0] +
            data[0][1] * rhs.data[1][0] +
            data[0][2] * rhs.data[2][0];
        tmp.data[1][0] =
            data[1][0] * rhs.data[0][0] +
            data[1][1] * rhs.data[1][0] +
            data[1][2] * rhs.data[2][0];
        tmp.data[2][0] =
            data[2][0] * rhs.data[0][0] +
            data[2][1] * rhs.data[1][0] +
            data[2][2] * rhs.data[2][0];
        tmp.data[0][1] =
            data[0][0] * rhs.data[0][1] +
            data[0][1] * rhs.data[1][1] +
            data[0][2] * rhs.data[2][1];
        tmp.data[1][1] =
            data[1][0] * rhs.data[0][1] +
            data[1][1] * rhs.data[1][1] +
            data[1][2] * rhs.data[2][1];
        tmp.data[2][1] =
            data[2][0] * rhs.data[0][1] +
            data[2][1] * rhs.data[1][1] +
            data[2][2] * rhs.data[2][1];
        tmp.data[0][2] =
            data[0][0] * rhs.data[0][2] +
            data[0][1] * rhs.data[1][2] +
            data[0][2] * rhs.data[2][2];
        tmp.data[1][2] =
            data[1][0] * rhs.data[0][2] +
            data[1][1] * rhs.data[1][2] +
            data[1][2] * rhs.data[2][2];
        tmp.data[2][2] =
            data[2][0] * rhs.data[0][2] +
            data[2][1] * rhs.data[1][2] +
            data[2][2] * rhs.data[2][2];
        return tmp;
    }

    NiMatrix33 NiMatrix33::operator* (double scalar) const
    {
        NiMatrix33 result;
        result.data[0][0] = data[0][0] * scalar;
        result.data[0][1] = data[0][1] * scalar;
        result.data[0][2] = data[0][2] * scalar;
        result.data[1][0] = data[1][0] * scalar;
        result.data[1][1] = data[1][1] * scalar;
        result.data[1][2] = data[1][2] * scalar;
        result.data[2][0] = data[2][0] * scalar;
        result.data[2][1] = data[2][1] * scalar;
        result.data[2][2] = data[2][2] * scalar;
        return result;
    }

    NiPoint3 NiMatrix33::operator* (const NiPoint3 &pt) const
    {
        return NiPoint3
        (
            data[0][0] * pt.x + data[0][1] * pt.y + data[0][2] * pt.z,
            data[1][0] * pt.x + data[1][1] * pt.y + data[1][2] * pt.z,
            data[2][0] * pt.x + data[2][1] * pt.y + data[2][2] * pt.z
        );
    }

    NiMatrix33 NiMatrix33::Transpose() const
    {
        NiMatrix33 result;
        result.data[0][0] = data[0][0];
        result.data[0][1] = data[1][0];
        result.data[0][2] = data[2][0];
        result.data[1][0] = data[0][1];
        result.data[1][1] = data[1][1];
        result.data[1][2] = data[2][1];
        result.data[2][0] = data[0][2];
        result.data[2][1] = data[1][2];
        result.data[2][2] = data[2][2];
        return result;
    }

    NiTransform::NiTransform()
    {
        rot.Identity();
        scale = 1.0;
    }

    NiTransform::NiTransform(const ::NiTransform &transformSingle)
    {
        rot = transformSingle.rot;
        pos = transformSingle.pos;
        scale = transformSingle.scale;
    }

    ::NiTransform NiTransform::ToSingle() const
    {
        ::NiTransform tmp;
        tmp.rot = rot.ToSingle();
        tmp.pos = pos.ToSingle();
        tmp.scale = scale;
        return tmp;
    }

    NiTransform NiTransform::operator*(const NiTransform &rhs) const
    {
        NiTransform tmp;
        tmp.scale = scale * rhs.scale;
        tmp.rot = rot * rhs.rot;
        tmp.pos = pos + (rot * rhs.pos) * scale;
        return tmp;
    }

    NiPoint3 NiTransform::operator*(const NiPoint3 &pt) const
    {
        return (((rot * pt) * scale) + pos);
    }

    void NiTransform::Invert(NiTransform &kDest) const
    {
        kDest.rot = rot.Transpose();
        kDest.scale = 1.0 / scale;
        kDest.pos = (kDest.rot * -pos) * kDest.scale;
    }

    struct NodeTransforms
    {
        NiTransform local;
        NiTransform world;
    };

    std::unordered_map<NiAVObject *, NodeTransforms> g_nodeTransforms;

    NiTransform GetLocalTransformForDesiredWorldTransform(NiAVObject *node, const NiTransform &worldTransform)
    {
        if (NiPointer<NiNode> parent = node->m_parent) {
            if (!g_nodeTransforms.contains(parent)) {
                g_nodeTransforms[parent] = { parent->m_localTransform, parent->m_worldTransform };
            }
            NiTransform inverseParent = InverseTransform(g_nodeTransforms[parent].world);
            return inverseParent * worldTransform;
        }
        return worldTransform;
    }

    void UpdateNodeTransformLocal(NiAVObject *node, const NiTransform &worldTransform)
    {
        // Given world transform, set the necessary local transform
        NiTransform local = GetLocalTransformForDesiredWorldTransform(node, worldTransform);
        ::NiTransform localSingle = local.ToSingle();
        //node->m_localTransform = localSingle;
        if (!g_nodeTransforms.contains(node)) {
            g_nodeTransforms[node] = { local, node->m_worldTransform };
        }
        else {
            g_nodeTransforms[node].local = local;
        }
    }

    void UpdateBSFlattenedBoneTree(BSFlattenedBoneTree *tree)
    {
        for (int i = 0; i < tree->numBones; i++) {
            BSFlattenedBoneTree::BoneEntry &entry = tree->boneEntries[i];
            if (entry.node) {
                entry.world = entry.node->m_worldTransform;
            }
            else {
                SInt16 parentIndex = entry.unk68;
                NiTransform parentWorld = parentIndex >= 0 ? tree->boneEntries[parentIndex].world : tree->m_worldTransform; // TODO: Should this wait until all bone entry world transforms are updated?
                NiTransform entryWorld = parentWorld * entry.local;
                entry.world = entryWorld.ToSingle();
            }
        }
    }

    void UpdateNodeImpl(NiAVObject *node, NiTransform *a_parentTransform = nullptr)
    {
        NiTransform parentTransform;
        if (a_parentTransform) {
            parentTransform = *a_parentTransform;
        }
        else if (node->m_parent) {
            if (!g_nodeTransforms.contains(node->m_parent)) {
                g_nodeTransforms[node->m_parent] = { node->m_parent->m_localTransform, node->m_parent->m_worldTransform };
            }
            parentTransform = g_nodeTransforms[node->m_parent].world;
        }

        if (!g_nodeTransforms.contains(node)) {
            g_nodeTransforms[node] = { node->m_localTransform, node->m_worldTransform };
        }

        NiTransform newTransform = parentTransform * g_nodeTransforms[node].local;
        g_nodeTransforms[node].world = newTransform;

        auto &transforms = g_nodeTransforms[node];
        node->m_localTransform = transforms.local.ToSingle();
        node->m_worldTransform = transforms.world.ToSingle();

        if (NiNode *asNode = node->GetAsNiNode()) {
            for (int i = 0; i < asNode->m_children.m_emptyRunStart; i++) {
                if (NiAVObject *child = asNode->m_children.m_data[i]) {
                    UpdateNodeImpl(child, &newTransform);
                }
            }
        }

        // Do this after regular NiNode stuff
        if (BSFlattenedBoneTree *tree = DYNAMIC_CAST(node, NiAVObject, BSFlattenedBoneTree)) {
            UpdateBSFlattenedBoneTree(tree);
        }
    }

    void UpdateNode(NiAVObject *node)
    {
        g_nodeTransforms.clear();
        UpdateNodeImpl(node);
    }

    void UpdateTransform(NiAVObject *node, const NiTransform &transform)
    {
        g_nodeTransforms.clear();

        UpdateNodeTransformLocal(node, transform);
        UpdateNodeImpl(node);
    }

    NiTransform UpdateClavicleToTransformHand(NiAVObject *a_clavicle, NiAVObject *a_hand, NiTransform *a_wandNodeTransformWorld, NiTransform *a_magicHandTransformLocal)
    {
        NiTransform v13; // [rsp+20h] [rbp-19h] BYREF

        if (!a_hand || !a_clavicle) return NiTransform{};

        g_nodeTransforms.clear();

        if (!g_nodeTransforms.contains(a_hand)) {
            g_nodeTransforms[a_hand] = { a_hand->m_localTransform, a_hand->m_worldTransform };
        }
        NiTransform handLocal = g_nodeTransforms[a_hand].local;

        NiTransform v14 = InverseTransform(*a_magicHandTransformLocal);
        v13 = handLocal * v14;

        NiNode *parent = a_hand->m_parent;
        while (parent && parent != a_clavicle) {
            if (!g_nodeTransforms.contains(parent)) {
                g_nodeTransforms[parent] = { parent->m_localTransform, parent->m_worldTransform };
            }
            NiTransform nodeLocal = g_nodeTransforms[parent].local;
            v13 = nodeLocal * v13;
            parent = parent->m_parent;
        }
        v14 = InverseTransform(v13);
        v13 = *a_wandNodeTransformWorld * v14;

        UpdateNodeTransformLocal(a_clavicle, v13);

        UpdateNodeImpl(a_clavicle);

        return g_nodeTransforms[a_hand].world;
    }
}

