#include <vector>

#include "skse64/GameTypes.h"
#include "skse64/NiTypes.h"


void StartGenerateFingerCurve(bool isLeft);
void StopGenerateFingerCurve();
void UpdateGenerateFingerCurve(BSFixedString &handNodeName, BSFixedString fingerNodeNames[5][3], float fingerTipLengths[5]);


extern NiPoint3 g_fingerZeroAngleVecs[5];
extern NiPoint3 g_fingerNormals[5];
extern float g_fingerCurveVals[5][201][3];