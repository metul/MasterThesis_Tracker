#include "tracking_export.h"
#include <string>
#include <rbgt/tracker.h>

extern "C"
{
int TRACKING_EXPORT run();
void TRACKING_EXPORT startTracking();
rbgt::objects TRACKING_EXPORT getData();
rbgt::markerPos TRACKING_EXPORT getMarker();
void TRACKING_EXPORT endTracking();
int TRACKING_EXPORT calibrate();
int TRACKING_EXPORT takeImages();
}