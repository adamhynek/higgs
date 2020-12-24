#pragma once

#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>

#include "skse64/GameData.h"


// Game Types //

struct BSEffectShaderData
{
	inline UInt32 IncRef() { return InterlockedIncrement(&refCount); }
	inline UInt32 DecRef() { return InterlockedDecrement(&refCount); }

	UInt32 refCount; // 00

	// more...
};

struct OwnedController
{
	void *vtbl; // 00
	UInt32 targetHandle; // 08
	UInt32 unk0C;
	TESEffectShader *shader; // 10
	BGSArtObject *artObject; // 18
	UInt32 aimAtHandle; // 20
	UInt32 unk24;
	NiPointer<NiAVObject> attachRoot; // 28
};
static_assert(sizeof(OwnedController) == 0x30);

struct BSTempEffect : NiObject
{
	float		   lifetime;	 // 10
	UInt32		   pad14;		 // 14
	TESObjectCELL* cell;		 // 18
	float		   age;			 // 20
	bool		   initialized;	 // 24
	UInt8		   pad25;		 // 25
	UInt16		   pad26;		 // 26
	UInt32		   effectID;	 // 28
	UInt32		   pad2C;		 // 2C
};
static_assert(sizeof(BSTempEffect) == 0x30);

struct ReferenceEffect : BSTempEffect
{
	OwnedController *	controller;		// 30
	UInt32				target;			// 38
	UInt32				aimAtTarget;	// 3C
	bool				finished;		// 40
	bool				ownController;	// 41
	UInt16				pad42;			// 42
	UInt32				pad44;			// 44
};
static_assert(offsetof(ReferenceEffect, controller) == 0x30);

struct ShaderReferenceEffect : ReferenceEffect
{
	UInt8 unkArraysAndSoundHandle[0xCC - 0x48]; // 48
	std::uint32_t		  pad0CC;			 // 0CC
	void*				  unk0D0;			 // 0D0 - smart ptr
	void*				  unk0D8;			 // 0D8 - smart ptr
	void*				  unk0E0;			 // 0E0 - smart ptr
	void*				  unk0E8;			 // 0E8 - smart ptr
	void*				  unk0F0;			 // 0F0 - smart ptr
	NiPointer<NiAVObject> lastRootNode;		 // 0F8
	TESBoundObject*		  wornObject;		 // 100
	TESEffectShader*	  effectData;		 // 108
	BSEffectShaderData*	  effectShaderData;	 // 110
	void*				  unk118;			 // 118 - smart ptr
	std::uint32_t		  unk120;			 // 120
	std::uint32_t		  unk124;			 // 124
	std::uint32_t		  unk128;			 // 128
	std::uint32_t		  unk12C;			 // 12C
	std::uint32_t		  flags;			 // 130
	std::uint32_t		  pushCount;		 // 134
};
static_assert(offsetof(ShaderReferenceEffect, pushCount) == 0x134);

struct ModelReferenceEffect : ReferenceEffect
{
	UInt8 otherBaseClassesAndHitEffectArtData[0xB0 - 0x48];  // 48
	std::uint64_t			unkB0;			   // B0
	BGSArtObject*			artObject;		   // B8
	std::uint64_t			unkC0;			   // C0
	NiPointer<NiAVObject>	artObject3D;	   // C8
	std::uint64_t			unkD0;			   // D0
};
static_assert(offsetof(ModelReferenceEffect, artObject3D) == 0xC8);

struct ProcessLists
{
	UInt8 unk00[0x108];
	tArray<NiPointer<BSTempEffect>> magicEffects; // 108
	SimpleLock magicEffectsLock; // 120
	// more...
};
static_assert(offsetof(ProcessLists, magicEffects) == 0x108);
static_assert(offsetof(ProcessLists, magicEffectsLock) == 0x120);


extern ShaderReferenceEffect ** volatile g_shaderReferenceToSet;

extern std::unordered_map<ShaderReferenceEffect *, std::unordered_set<BSGeometry *>> *g_shaderNodes; // Map root node to set of geom off of that root
extern std::shared_mutex g_shaderNodesLock;


// My Types //

struct PlayingShader
{
	TESEffectShader *shader;
	NiPointer<ShaderReferenceEffect> shaderReference;
	UInt32 handle;
	NiPointer<NiAVObject> node;

	bool IsPlaying() const;
};
void PlayShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader, bool saveCurrentShader);
void StopShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader, bool restoreCurrentShader);
