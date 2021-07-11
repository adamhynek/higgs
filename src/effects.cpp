#include "RE/offsets.h"
#include "effects.h"
#include "utils.h"
#include "hand.h"

#include "skse64/NiGeometry.h"
#include "skse64/GameRTTI.h"


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

void ClearEffectDataMap()
{
	if (!g_playingShaders[0].shaderReference && !g_playingShaders[1].shaderReference) {
		if (g_effectDataMap->size() > 0) {
			g_effectDataMap->clear();
		}
	}
}

void SaveShaderData(UInt32 handle, NiAVObject *root)
{
	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		auto shaderProperty = DYNAMIC_CAST(geom->m_spEffectState, NiProperty, BSLightingShaderProperty);
		if (shaderProperty) {
			NiPointer<BSEffectShaderData> effectData = (BSEffectShaderData *)shaderProperty->unk68;
			if (effectData) {
				ProcessLists *processLists = *g_processLists;
				{
					SimpleLocker locker(&processLists->magicEffectsLock);

					for (int i = 0; i < processLists->magicEffects.count; i++) {
						NiPointer<BSTempEffect> effect = processLists->magicEffects.entries[i];
						ShaderReferenceEffect *shaderReference = DYNAMIC_CAST(effect, BSTempEffect, ShaderReferenceEffect);
						if (shaderReference) {
							// We only care about saving shader data if it's a shader effect
							NiPointer<NiAVObject> attachRoot = (shaderReference->ownController && shaderReference->controller) ? shaderReference->controller->attachRoot : nullptr;
							if ((shaderReference->target == handle && !attachRoot) || attachRoot == root) {
								if (shaderReference->effectShaderData == effectData) { // Make sure we save the shader that's actually affecting the node
									TESEffectShader *shader = shaderReference->effectData;
									if (shader != g_rightHand->itemSelectedShader && shader != g_rightHand->itemSelectedShaderOffLimits) {
										// Only save shader data for shaders that are not our own
										auto &effectDataMap = *g_effectDataMap;
										effectDataMap[root] = shaderReference;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				SaveShaderData(handle, child);
			}
		}
	}
}

void RestoreShaderData(UInt32 handle, NiAVObject *root)
{
	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		auto shaderProperty = DYNAMIC_CAST(geom->m_spEffectState, NiProperty, BSShaderProperty);
		if (shaderProperty) {
			auto &effectDataMap = *g_effectDataMap;
			if (effectDataMap.count(root) != 0) {
				NiPointer<ShaderReferenceEffect> savedShaderReference = effectDataMap[root];

				// Make sure the shader that was playing when we saved the shader data is still playing now that we're restoring its data
				ProcessLists *processLists = *g_processLists;
				{
					SimpleLocker locker(&processLists->magicEffectsLock);

					for (int i = 0; i < processLists->magicEffects.count; i++) {
						NiPointer<BSTempEffect> effect = processLists->magicEffects.entries[i];
						ShaderReferenceEffect *shaderReference = DYNAMIC_CAST(effect, BSTempEffect, ShaderReferenceEffect);
						if (shaderReference) {
							if (shaderReference->target == handle && shaderReference == savedShaderReference) {
								NiPointer<BSEffectShaderData> shaderData = shaderReference->effectShaderData;
								if (shaderData) {
									*((NiPointer<BSEffectShaderData> *)&shaderProperty->unk68) = shaderData;
									effectDataMap.erase(root);
								}
							}
						}
					}
				}
			}
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				RestoreShaderData(handle, child);
			}
		}
	}
}


std::shared_mutex g_shaderNodesLock;


const size_t g_vtblSize = 0x40; // 0x3F vfuncs + typeinfo above vtable
uintptr_t g_vtblCopy[g_vtblSize];
uintptr_t *g_vfuncsCopy = g_vtblCopy + 1;
uintptr_t g_shaderReferenceEffectDtor = 0;
bool g_isVtblCopied = false;

typedef void(*_ShaderReferenceEffectDtor)(ShaderReferenceEffect *_this);
void HookedShaderReferenceEffectDtor(ShaderReferenceEffect *_this)
{
	{
		// Clear the nodes from the map for this reference effect
		std::unique_lock lock(g_shaderNodesLock);
		g_shaderNodes->erase(_this);
	}

	((_ShaderReferenceEffectDtor)g_shaderReferenceEffectDtor)(_this);
}

void FillGeometryNodes(NiAVObject *root, std::unordered_set<BSGeometry *> &geometryNodes, bool terminateAtCollision)
{
	// Populate geometry nodes until we hit nodes with their own collision

	BSGeometry *geom = root->GetAsBSGeometry();
	if (geom) {
		auto shaderProperty = DYNAMIC_CAST(geom->m_spEffectState, NiProperty, BSShaderProperty);
		if (shaderProperty) {
			if (shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_Facegen_Detail_Map ||
				shaderProperty->shaderFlags1 & BSShaderProperty::ShaderFlags1::kSLSF1_FaceGen_RGB_Tint) {

				// It's skin geometry, so don't play the shader on it
			}
			else {
				geometryNodes.insert(geom);
			}
		}
	}

	NiNode *node = root->GetAsNiNode();
	if (node) {
		for (int i = 0; i < node->m_children.m_emptyRunStart; i++) {
			NiAVObject *child = node->m_children.m_data[i];
			if (child) {
				auto rigidBody = GetRigidBody(child);
				bool isSelected = rigidBody == g_rightHand->selectedObject.rigidBody || rigidBody == g_leftHand->selectedObject.rigidBody;
				// Do not play the shader on nodes that have collision and have a motion type that actually makes them interact with physics
				// If the rigidbody does not interact with physics, if one of the hands is holding it, do not play the shader on it
				if (!rigidBody || (!IsMoveableEntity(rigidBody->hkBody) && !isSelected) || !terminateAtCollision) {
					FillGeometryNodes(child, geometryNodes, terminateAtCollision);
				}
			}
		}
	}
}

ShaderReferenceEffect * PlayEffectShader(TESEffectShader *shader, TESObjectREFR *refr)
{
	ShaderReferenceEffect *shaderReference;

	g_shaderReferenceToSet = &shaderReference;
	EffectShader_Play(VM_REGISTRY, 0, shader, refr, 60.0f);
	g_shaderReferenceToSet = nullptr;

	if (!g_isVtblCopied) {
		g_isVtblCopied = true;

		// Copy the vtable
		uintptr_t *vtbl = *((uintptr_t **)shaderReference);
		memcpy(g_vfuncsCopy - 1, vtbl - 1, g_vtblSize * sizeof(uintptr_t)); // Need to copy the typeinfo as well as the vfuncs

		// Replace the destructor with our hooked one
		g_shaderReferenceEffectDtor = g_vfuncsCopy[0];
		g_vfuncsCopy[0] = (uintptr_t)HookedShaderReferenceEffectDtor;
	}

	// Replace the shader reference's vtable with the copied/modified one
	*((UInt64 **)shaderReference) = ((UInt64 *)(g_vfuncsCopy));

	return shaderReference;
}

void CommitShaderNodes(ShaderReferenceEffect *shaderReference, NiAVObject *node, bool terminateAtCollision)
{
	// The nodes that get put into the set are the ones that actually get the shader to play on them

	std::unordered_set<BSGeometry *> geometryNodes;
	FillGeometryNodes(node, geometryNodes, terminateAtCollision);

	std::unique_lock lock(g_shaderNodesLock);
	(*g_shaderNodes)[shaderReference] = std::move(geometryNodes);
}

void PlayShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader, bool saveCurrentShader)
{
	bool isFirstShaderPlaying = g_playingShaders[0].IsPlaying();
	bool isSecondShaderPlaying = g_playingShaders[1].IsPlaying();

	if (isFirstShaderPlaying && isSecondShaderPlaying) {
		// Both shaders are already playing
		return;
	}

	if (!isFirstShaderPlaying && !isSecondShaderPlaying) {
		// No shaders in use - use the first one
		NiPointer<TESObjectREFR> obj;
		UInt32 handleCopy = objHandle;
		if (LookupREFRByHandle(handleCopy, obj)) {
			NiPointer<NiNode> objRoot = obj->GetNiNode();
			if (objRoot) {
				if (node) {
					if (saveCurrentShader) {
						SaveShaderData(objHandle, node);
					}
				}

				PlayingShader &freeShader = g_playingShaders[0];

				freeShader.handle = objHandle;
				freeShader.shader = shader;

				freeShader.shaderReference = PlayEffectShader(freeShader.shader, obj);
				freeShader.shaderReference->controller->attachRoot = node;

				CommitShaderNodes(freeShader.shaderReference, node ? node : objRoot, node);

				freeShader.node = node;
			}
		}

		return;
	}

	// One shader is playing, and the other is not
	PlayingShader &freeShader = isFirstShaderPlaying ? g_playingShaders[1] : g_playingShaders[0];
	PlayingShader &playingShader = isFirstShaderPlaying ? g_playingShaders[0] : g_playingShaders[1];

	// Set the params for the shader, but only actually play the shader if it's not already playing
	freeShader.handle = objHandle;
	freeShader.node = node;

	if (node) {
		if (playingShader.node != node) {
			// Other shader is playing on another node - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				if (saveCurrentShader) {
					SaveShaderData(objHandle, node);
				}

				freeShader.shader = shader;

				freeShader.shaderReference = PlayEffectShader(freeShader.shader, obj);
				freeShader.shaderReference->controller->attachRoot = node;

				CommitShaderNodes(freeShader.shaderReference, node, true);
			}
		}
	}
	else {
		if (playingShader.handle != objHandle || playingShader.node) {
			// Other shader is playing on another object, or a specific node on this object - play the shader
			NiPointer<TESObjectREFR> obj;
			UInt32 handleCopy = objHandle;
			if (LookupREFRByHandle(handleCopy, obj)) {
				NiPointer<NiNode> objRoot = obj->GetNiNode();
				if (objRoot) {
					freeShader.shader = shader;

					freeShader.shaderReference = PlayEffectShader(freeShader.shader, obj);;
					freeShader.shaderReference->controller->attachRoot = node; // node is null

					CommitShaderNodes(freeShader.shaderReference, objRoot, false);
				}
			}
		}
	}
}

void StopShader(UInt32 objHandle, NiAVObject *node, TESEffectShader *shader, bool restoreCurrentShader)
{
	bool isFirstShaderPlaying = g_playingShaders[0].IsPlaying();
	bool isSecondShaderPlaying = g_playingShaders[1].IsPlaying();

	if (!isFirstShaderPlaying && !isSecondShaderPlaying) {
		ClearEffectDataMap();
		return;
	}

	bool isFirstShaderTheOne = g_playingShaders[0].handle == objHandle && g_playingShaders[0].node == node;
	bool isSecondShaderTheOne = g_playingShaders[1].handle == objHandle && g_playingShaders[1].node == node;

	if (!isFirstShaderTheOne && !isSecondShaderTheOne) {
		// Neither shader is playing for the given refr / node
		return;
	}
	else if (isFirstShaderTheOne && isSecondShaderTheOne) {
		// Clear one of the shaders - whichever does not have the shader actually playing
		if (g_playingShaders[1].shaderReference) {
			g_playingShaders[0].shader = nullptr;
			g_playingShaders[0].handle = *g_invalidRefHandle;
			g_playingShaders[0].node = nullptr;
		}
		else {
			g_playingShaders[1].shader = nullptr;
			g_playingShaders[1].handle = *g_invalidRefHandle;
			g_playingShaders[1].node = nullptr;
		}
		return;
	}

	// Only one shader is on the refr / node. Stop the shader.
	PlayingShader &neo = isFirstShaderTheOne ? g_playingShaders[0] : g_playingShaders[1];
	if ((isFirstShaderTheOne && isFirstShaderPlaying) || (isSecondShaderTheOne && isSecondShaderPlaying)) {
		neo.shaderReference->finished = true; // This is all it takes to stop the shader

		if (neo.node) {
			if (restoreCurrentShader) {
				RestoreShaderData(objHandle, neo.node);
			}
		}
	}
	
	neo.shaderReference = nullptr;
	neo.shader = nullptr;
	neo.handle = *g_invalidRefHandle;
	neo.node = nullptr;

	ClearEffectDataMap();
}
