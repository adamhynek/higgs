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


extern NiPoint3 g_fingerZeroAngleVecs[6];
extern NiPoint3 g_fingerNormals[6];
extern NiPoint3 g_fingerStartPositions[6];
extern SavedFingerData g_fingerTipVals[6][201];
extern SavedFingerData g_fingerOuterVals[6][201];
extern SavedFingerData g_fingerInnerVals[6][201];
constexpr int g_numFingerVals = std::size(g_fingerTipVals[0]);
