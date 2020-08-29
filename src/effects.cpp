#include "effects.h"
#include "utils.h"
#include "offsets.h"


PlayingShader _shaders[2];
PlayingEffect _effects[2];


bool IsEffectPlaying(ModelReferenceEffect *effect)
{
	return _effects[0].modelReference == effect || _effects[1].modelReference == effect;
}


bool PlayingEffect::IsPlaying() const
{
	// Return true iff the shader was started and the obj / node on which it plays still exists
	if (!modelReference) {
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

void PlayVFX(UInt32 objHandle, NiAVObject *node, BGSReferenceEffect *effect)
{
	bool isFirstEffectPlaying = _effects[0].IsPlaying();
	bool isSecondEffectPlaying = _effects[1].IsPlaying();

	if (isFirstEffectPlaying && isSecondEffectPlaying) {
		// Both shaders are already playing
		return;
	}

	if (!isFirstEffectPlaying && !isSecondEffectPlaying) {
		// No shaders in use - use the first one
		NiPointer<TESObjectREFR> obj;
		UInt32 handleCopy = objHandle;
		if (LookupREFRByHandle(handleCopy, obj)) {
			PlayingEffect &freeEffect = _effects[0];

			freeEffect.handle = objHandle;
			freeEffect.effect = effect;

			ModelReferenceEffect *modelReference;
			g_modelReferenceToSet = &modelReference;
			VisualEffect_Play(VM_REGISTRY, 0, effect, obj, 60.0f, *g_thePlayer);
			g_modelReferenceToSet = nullptr;
			freeEffect.modelReference = modelReference;

			freeEffect.node = node;
			freeEffect.modelReference->controller->attachRoot = node;
		}

		return;
	}

	// One shader is playing, and the other is not
	PlayingEffect &freeEffect = isFirstEffectPlaying ? _effects[1] : _effects[0];
	PlayingEffect &playingEffect = isFirstEffectPlaying ? _effects[0] : _effects[1];

	// Set the params for the shader, but only actually play the shader if it's not already playing
	freeEffect.handle = objHandle;
	freeEffect.node = node;

	if (node) {
		if (playingEffect.node != node) {
			// Other shader is playing on another node - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				freeEffect.effect = effect;

				ModelReferenceEffect *modelReference;
				g_modelReferenceToSet = &modelReference;
				VisualEffect_Play(VM_REGISTRY, 0, effect, obj, 60.0f, *g_thePlayer);
				g_modelReferenceToSet = nullptr;
				freeEffect.modelReference = modelReference;

				freeEffect.modelReference->controller->attachRoot = node;
			}
		}
	}
	else {
		if (playingEffect.handle != objHandle || playingEffect.node) {
			// Other shader is playing on another object, or a specific node on this object - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				freeEffect.effect = effect;

				ModelReferenceEffect *modelReference;
				g_modelReferenceToSet = &modelReference;
				VisualEffect_Play(VM_REGISTRY, 0, effect, obj, 60.0f, *g_thePlayer);
				g_modelReferenceToSet = nullptr;
				freeEffect.modelReference = modelReference;

				freeEffect.modelReference->controller->attachRoot = node;
			}
		}
	}
}

void StopVFX(UInt32 objHandle, NiAVObject *node, BGSReferenceEffect *effect)
{
	bool isFirstEffectPlaying = _effects[0].IsPlaying();
	bool isSecondEffectPlaying = _effects[1].IsPlaying();

	if (!isFirstEffectPlaying && !isSecondEffectPlaying) {
		return;
	}

	bool isFirstEffectTheOne = _effects[0].handle == objHandle && _effects[0].node == node;
	bool isSecondEffectTheOne = _effects[1].handle == objHandle && _effects[1].node == node;

	if (!isFirstEffectTheOne && !isSecondEffectTheOne) {
		// Neither shader is playing for the given refr / node
		return;
	}
	else if (isFirstEffectTheOne && isSecondEffectTheOne) {
		// Clear one of the shaders - whichever does not have the shader actually playing
		if (_effects[1].modelReference) {
			_effects[0].effect = nullptr;
			_effects[0].handle = *g_invalidRefHandle;
			_effects[0].node = nullptr;
		}
		else {
			_effects[1].effect = nullptr;
			_effects[1].handle = *g_invalidRefHandle;
			_effects[1].node = nullptr;
		}
		return;
	}

	// Only one shader is on the refr / node. Stop the shader.
	PlayingEffect &neo = isFirstEffectTheOne ? _effects[0] : _effects[1];
	if ((isFirstEffectTheOne && isFirstEffectPlaying) || (isSecondEffectTheOne && isSecondEffectPlaying)) {
		neo.modelReference->finished = true; // This is all it takes to stop the shader
	}

	neo.modelReference = nullptr;
	neo.effect = nullptr;
	neo.handle = *g_invalidRefHandle;
	neo.node = nullptr;
}


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
	}
	
	neo.shaderReference = nullptr;
	neo.shader = nullptr;
	neo.handle = *g_invalidRefHandle;
	neo.node = nullptr;
}
