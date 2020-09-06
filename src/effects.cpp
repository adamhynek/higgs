#include "effects.h"
#include "utils.h"
#include "offsets.h"

#include "skse64/NiGeometry.h"
#include "skse64/GameRTTI.h"


PlayingShader _shaders[2];


bool PlayingShader::IsPlaying() const
{
	// Return true iff the shader was started and the obj / node on which it plays still exists
	if (!shaderReference) {
		return false;
	}

	NiPointer<TESObjectREFR> obj;
	UInt32 handleCopy = handle; // handle can get set to 0 by LookupREFRByHandle
	bool doesObjExist = LookupREFRByHandle(handleCopy, obj);
	if (doesObjExist && node) {
		return DoesRefrHaveNode(obj, node);
	}
	return doesObjExist;
}

struct BSEffectShaderData
{
	inline UInt32 IncRef() { return InterlockedIncrement(&refCount); }
	inline UInt32 DecRef() { return InterlockedDecrement(&refCount); }

	UInt32 refCount; // 00

	// more...
};


std::unordered_map<NiAVObject *, NiPointer<BSEffectShaderData>> effectDataMap;

void ClearEffectDataMap()
{
	if (!_shaders[0].shaderReference && !_shaders[1].shaderReference) {
		if (effectDataMap.size() > 0) {
			effectDataMap.clear();
		}
	}
}

void SaveShaderData(NiAVObject *root)
{
	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		auto shaderProperty = DYNAMIC_CAST(geom->m_spEffectState, NiProperty, BSLightingShaderProperty);
		if (shaderProperty) {
			NiPointer<BSEffectShaderData> effectData = (BSEffectShaderData *)shaderProperty->unk68;
			if (effectData) {
				effectDataMap[root] = effectData;
			}
			_MESSAGE("%s: %x", root->m_name ? root->m_name : "", effectData);
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				SaveShaderData(child);
			}
		}
	}
}

void RestoreShaderData(NiAVObject *root)
{
	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		auto shaderProperty = DYNAMIC_CAST(geom->m_spEffectState, NiProperty, BSLightingShaderProperty);
		if (shaderProperty) {
			if (effectDataMap.count(root) != 0) {
				NiPointer<BSEffectShaderData> shaderData = effectDataMap[root];
				shaderData->IncRef();
				*((BSEffectShaderData **)&shaderProperty->unk68) = shaderData;
				effectDataMap.erase(root);
			}
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				RestoreShaderData(child);
			}
		}
	}
}

void PlayShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader)
{
	bool isFirstShaderPlaying = _shaders[0].IsPlaying();
	bool isSecondShaderPlaying = _shaders[1].IsPlaying();

	if (isFirstShaderPlaying && isSecondShaderPlaying) {
		// Both shaders are already playing
		return;
	}

	if (!isFirstShaderPlaying && !isSecondShaderPlaying) {
		// No shaders in use - use the first one
		NiPointer<TESObjectREFR> obj;
		UInt32 handleCopy = objHandle;
		if (LookupREFRByHandle(handleCopy, obj)) {
			if (node) {
				SaveShaderData(node);
			}

			PlayingShader &freeShader = _shaders[0];

			freeShader.handle = objHandle;
			freeShader.shader = shader;

			ShaderReferenceEffect *shaderReference;
			g_shaderReferenceToSet = &shaderReference;
			EffectShader_Play(VM_REGISTRY, 0, freeShader.shader, obj, 60.0f);
			g_shaderReferenceToSet = nullptr;
			freeShader.shaderReference = shaderReference;

			freeShader.node = node;
			freeShader.shaderReference->controller->attachRoot = node;
		}

		return;
	}

	// One shader is playing, and the other is not
	PlayingShader &freeShader = isFirstShaderPlaying ? _shaders[1] : _shaders[0];
	PlayingShader &playingShader = isFirstShaderPlaying ? _shaders[0] : _shaders[1];

	// Set the params for the shader, but only actually play the shader if it's not already playing
	freeShader.handle = objHandle;
	freeShader.node = node;

	if (node) {
		if (playingShader.node != node) {
			// Other shader is playing on another node - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				SaveShaderData(node);

				freeShader.shader = shader;

				ShaderReferenceEffect *shaderReference;
				g_shaderReferenceToSet = &shaderReference;
				EffectShader_Play(VM_REGISTRY, 0, freeShader.shader, obj, 60.0f);
				g_shaderReferenceToSet = nullptr;
				freeShader.shaderReference = shaderReference;

				freeShader.shaderReference->controller->attachRoot = node;
			}
		}
	}
	else {
		if (playingShader.handle != objHandle || playingShader.node) {
			// Other shader is playing on another object, or a specific node on this object - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				freeShader.shader = shader;

				ShaderReferenceEffect *shaderReference;
				g_shaderReferenceToSet = &shaderReference;
				EffectShader_Play(VM_REGISTRY, 0, freeShader.shader, obj, 60.0f);
				g_shaderReferenceToSet = nullptr;
				freeShader.shaderReference = shaderReference;

				freeShader.shaderReference->controller->attachRoot = node;
			}
		}
	}
}

void StopShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader)
{
	bool isFirstShaderPlaying = _shaders[0].IsPlaying();
	bool isSecondShaderPlaying = _shaders[1].IsPlaying();

	if (!isFirstShaderPlaying && !isSecondShaderPlaying) {
		ClearEffectDataMap();
		return;
	}

	bool isFirstShaderTheOne = _shaders[0].handle == objHandle && _shaders[0].node == node;
	bool isSecondShaderTheOne = _shaders[1].handle == objHandle && _shaders[1].node == node;

	if (!isFirstShaderTheOne && !isSecondShaderTheOne) {
		// Neither shader is playing for the given refr / node
		return;
	}
	else if (isFirstShaderTheOne && isSecondShaderTheOne) {
		// Clear one of the shaders - whichever does not have the shader actually playing
		if (_shaders[1].shaderReference) {
			_shaders[0].shader = nullptr;
			_shaders[0].handle = *g_invalidRefHandle;
			_shaders[0].node = nullptr;
		}
		else {
			_shaders[1].shader = nullptr;
			_shaders[1].handle = *g_invalidRefHandle;
			_shaders[1].node = nullptr;
		}
		return;
	}

	// Only one shader is on the refr / node. Stop the shader.
	PlayingShader &neo = isFirstShaderTheOne ? _shaders[0] : _shaders[1];
	if ((isFirstShaderTheOne && isFirstShaderPlaying) || (isSecondShaderTheOne && isSecondShaderPlaying)) {
		neo.shaderReference->finished = true; // This is all it takes to stop the shader

		if (neo.node) {
			RestoreShaderData(neo.node);
		}
	}
	
	neo.shaderReference = nullptr;
	neo.shader = nullptr;
	neo.handle = *g_invalidRefHandle;
	neo.node = nullptr;

	ClearEffectDataMap();
}