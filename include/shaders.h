#pragma once

#include "skse64/GameData.h"

struct OwnedController
{
	void *vtbl; // 00
	UInt32 targetHandle; // 08
	UInt32 unk0C;
	TESEffectShader *shader; // 10
	BGSArtObject *artObject; // 18
	UInt32 unkHandle; // 20
	UInt32 unk24;
	NiPointer<NiAVObject> attachRoot; // 28
};
static_assert(sizeof(OwnedController) == 0x30);

struct ShaderReferenceEffect : NiObject
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
	OwnedController *controller; // 30
	UInt32			   target;		   // 38
	UInt32			   aimAtTarget;	   // 3C
	bool					   finished;	   // 40
	bool					   ownController;  // 41
	UInt16					   pad42;		   // 42
	UInt32					   pad44;		   // 44
	// more...
};
static_assert(offsetof(ShaderReferenceEffect, controller) == 0x30);

extern ShaderReferenceEffect ** volatile g_shaderReferenceToSet;


struct PlayingShader
{
	TESEffectShader *shader;
	NiPointer<ShaderReferenceEffect> shaderReference;
	UInt32 handle;
	NiPointer<NiAVObject> node;

	bool IsPlaying() const;
};
void PlayShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader);
void StopShader(UInt32 objHandle, NiAVObject *node);
