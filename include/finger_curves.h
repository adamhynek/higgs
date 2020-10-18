#include <vector>

#include "skse64/GameTypes.h"
#include "skse64/NiTypes.h"


struct SavedFingerData
{
	float curveVal; // open/closed value. 0 == open, 1 == closed
	float angle; // rad
	float fingerLength;
};

void StartGenerateFingerCurve(bool isLeft);
void StopGenerateFingerCurve();
void UpdateGenerateFingerCurve(BSFixedString &handNodeName, BSFixedString fingerNodeNames[5][3]);

int LookupFingerByAngle(int fingerIndex, float desiredAngle, SavedFingerData *out);


extern NiPoint3 g_fingerZeroAngleVecs[5];
extern NiPoint3 g_fingerNormals[5];
extern SavedFingerData g_fingerCurveVals[5][201];
