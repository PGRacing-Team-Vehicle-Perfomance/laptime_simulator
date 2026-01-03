template <typename T>
struct vec3 {
    T x;
    T y;
    T z;
};

struct Body {
    float mass;
    vec3<float> massCenter;
};
