namespace MathLib
{
    struct alignas(16) Vec2f
    {
        union
        {
            __m128 v;
            float f[2];
            struct
            {
                float x, y;
            };
        };

        // --- Constructors ---
        Vec2f() : v(_mm_setzero_ps()) {}

        // Internal SIMD constructor: Blends input to ensure Z and W are always 0.0f
        Vec2f(__m128 val) : v(_mm_blend_ps(val, _mm_setzero_ps(), 0b1100)) {}

        Vec2f(float x, float y) : v(_mm_set_ps(0.0f, 0.0f, y, x)) {}

        // --- Arithmetic Operators ---
        friend Vec2f operator+(const Vec2f& a, const Vec2f& b)
        {
            return _mm_add_ps(a.v, b.v);
        }

        friend Vec2f operator-(const Vec2f& a, const Vec2f& b)
        {
            return _mm_sub_ps(a.v, b.v);
        }

        friend Vec2f operator*(const Vec2f& vec, float scalar)
        {
            return _mm_mul_ps(vec.v, _mm_set1_ps(scalar));
        }

        friend Vec2f operator/(const Vec2f& vec, float scalar)
        {
            return _mm_div_ps(vec.v, _mm_set1_ps(scalar));
        }

        // --- Math Functions ---
        static float dot(const Vec2f& a, const Vec2f& b)
        {
            // 0x31 mask: 
            // (0x3) multiply and sum elements 0 and 1 (x, y).
            // (0x1) store the sum in element 0 and zero out the rest.
            return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0x31));
        }

        float length_sqr() const
        {
            return dot(*this, *this);
        }

        float length() const
        {
            return std::sqrt(length_sqr());
        }

        Vec2f normalize() const
        {
            float len = length();
            if (len < 1e-6f)
                return Vec2f(0.0f, 0.0f);
            return *this / len;
        }
    };
}