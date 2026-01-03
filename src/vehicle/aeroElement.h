#include "vehicle/body.h"

struct AeroElement : public Body {
    float cla;
    vec3<float> claPosition;
};
