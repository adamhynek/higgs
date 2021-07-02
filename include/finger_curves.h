#include <vector>

#include "skse64/GameTypes.h"
#include "skse64/NiTypes.h"


struct SavedFingerData
{
	float curveVal; // open/closed value. 1 == open, 0 == closed
	float angle; // rad
	float fingerLength;
};

void StartGenerateFingerCurve(bool isLeft);
void StopGenerateFingerCurve();
void UpdateGenerateFingerCurve(BSFixedString &handNodeName, BSFixedString fingerNodeNames[5][3]);

int LookupFingerByAngle(SavedFingerData fingerVals[], float desiredAngle, SavedFingerData *out);



extern NiPoint3 g_openFingerPositions[5][3];
extern NiPoint3 g_closedFingerPositions[5][3];
extern NiQuaternion g_openFingerRotations[5][3];
extern NiQuaternion g_closedFingerRotations[5][3];

extern NiPoint3 g_fingerZeroAngleVecs[5];
extern NiPoint3 g_fingerNormals[5];
extern NiPoint3 g_fingerStartPositions[5];
extern SavedFingerData g_fingerTipVals[5][201];
extern SavedFingerData g_fingerOuterVals[5][201];
extern SavedFingerData g_fingerInnerVals[5][201];
constexpr int g_numFingerVals = std::size(g_fingerTipVals[0]);
