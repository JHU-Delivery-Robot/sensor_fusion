/**
 * @file FusionMath.h
 * @author Seb Madgwick
 * @brief Math library.
 */

#ifndef FUSION_MATH_HPP
#define FUSION_MATH_HPP

//------------------------------------------------------------------------------
// Includes

#include <data_types.hpp>
using fpm::sqrt;
using fpm::atan2;
using fpm::asin;
#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief 3D vector.
 */
union FusionVector {
    fixed_pt_num array[3];

    struct {
        fixed_pt_num x;
        fixed_pt_num y;
        fixed_pt_num z;
    } axis;

    FusionVector& operator=(const FusionVector& other) {
        array[0] = other.array[0];
        array[1] = other.array[1];
        array[2] = other.array[2];
        axis = other.axis;
        return *this;
    }
};

/**
 * @brief Quaternion.
 */
union FusionQuaternion {
    fixed_pt_num array[4];

    struct {
        fixed_pt_num w;
        fixed_pt_num x;
        fixed_pt_num y;
        fixed_pt_num z;
    } element;

    FusionQuaternion& operator=(const FusionQuaternion& other) {
        array[0] = other.array[0];
        array[1] = other.array[1];
        array[2] = other.array[2];
        array[4] = other.array[4];
        element = other.element;
        return *this;
    }
};

/**
 * @brief 3x3 matrix in row-major order.
 * See http://en.wikipedia.org/wiki/Row-major_order
 */
union FusionMatrix {
    fixed_pt_num array[3][3];

    struct {
        fixed_pt_num xx;
        fixed_pt_num xy;
        fixed_pt_num xz;
        fixed_pt_num yx;
        fixed_pt_num yy;
        fixed_pt_num yz;
        fixed_pt_num zx;
        fixed_pt_num zy;
        fixed_pt_num zz;
    } element;

    FusionMatrix& operator=(const FusionMatrix& other) {
        array[0][0] = other.array[0][0];
        array[0][1] = other.array[0][1];
        array[0][2] = other.array[0][2];
        array[1][0] = other.array[1][0];
        array[1][1] = other.array[1][1];
        array[1][2] = other.array[1][2];
        array[2][0] = other.array[2][0];
        array[2][1] = other.array[2][1];
        array[2][2] = other.array[2][2];
        element = other.element;
        return *this;
    }
};

/**
 * @brief Euler angles.  Roll, pitch, and yaw correspond to rotations around
 * X, Y, and Z respectively.
 */
union FusionEuler {
    fixed_pt_num array[3];

    struct {
        fixed_pt_num roll;
        fixed_pt_num pitch;
        fixed_pt_num yaw;
    } angle;

    FusionEuler& operator=(const FusionEuler& other) {
        array[0] = other.array[0];
        array[1] = other.array[1];
        array[2] = other.array[2];
        angle = other.angle;
        return *this;
    }
};

/**
 * @brief Vector of zeros.
 */
#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}} })

/**
 * @brief Vector of ones.
 */
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {fixed_pt_num{1.0}, fixed_pt_num{1.0}, fixed_pt_num{1.0}} })

/**
 * @brief Identity quaternion.
 */
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}} })

/**
 * @brief Identity matrix.
 */
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{fixed_pt_num{1.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}}, {fixed_pt_num{0.0}, fixed_pt_num{1.0}, fixed_pt_num{0.0}}, {fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{1.0}}} })

/**
 * @brief Euler angles of zero.
 */
#define FUSION_EULER_ZERO ((FusionEuler){ .array = {fixed_pt_num{0.0}, fixed_pt_num{0.0}, fixed_pt_num{0.0}} })

/**
 * @brief Include this definition or add as a preprocessor definition to use
 * normal square root operations.
 */
//#define FUSION_USE_NORMAL_SQRT

//------------------------------------------------------------------------------
// Inline functions - Degrees and radians conversion

/**
 * @brief Converts degrees to radians.
 * @param degrees Degrees.
 * @return Radians.
 */
static inline fixed_pt_num FusionDegreesToRadians(const fixed_pt_num degrees) {
    return degrees * (fixed_pt_num{0}.pi() / fixed_pt_num{180.0});
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline fixed_pt_num FusionRadiansToDegrees(const fixed_pt_num radians) {
    return radians * (fixed_pt_num{180.0} / fixed_pt_num{0}.pi());
}

//------------------------------------------------------------------------------
// Inline functions - Arc sine

/**
 * @brief Returns the arc sine of the value.
 * @param value Value.
 * @return Arc sine of the value.
 */
static inline fixed_pt_num FusionAsin(const fixed_pt_num value) {
    if (value <= fixed_pt_num{-1.0}) {
        return fixed_pt_num{0}.pi() / fixed_pt_num{-2.0};
    }
    if (value >= fixed_pt_num{1.0}) {
        return fixed_pt_num{0}.pi() / fixed_pt_num{2.0};
    }
    return asin(value);
}

//------------------------------------------------------------------------------
// Inline functions - Fast inverse square root

#ifndef FUSION_USE_NORMAL_SQRT

/**
 * @brief Calculates the reciprocal of the square root.
 * See https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
 * @param x Operand.
 * @return Reciprocal of the square root of x.
 */
static inline fixed_pt_num FusionFastInverseSqrt(const fixed_pt_num x) {

    typedef union {
        fixed_pt_num f;
        int32_t i;
    } Union32;

    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (fixed_pt_num{1.69000231} - fixed_pt_num{0.714158168} * x * union32.f * union32.f);
}

#endif

//------------------------------------------------------------------------------
// Inline functions - Vector operations

/**
 * @brief Returns true if the vector is zero.
 * @param vector Vector.
 * @return True if the vector is zero.
 */
static inline bool FusionVectorIsZero(const FusionVector vector) {
    return (vector.axis.x == fixed_pt_num{0.0}) && (vector.axis.y == fixed_pt_num{0.0}) && (vector.axis.z == fixed_pt_num{0.0});
}

/**
 * @brief Returns the sum of two vectors.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Sum of two vectors.
 */
static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
    FusionVector result;
    result.axis.x = vectorA.axis.x + vectorB.axis.x;
    result.axis.y = vectorA.axis.y + vectorB.axis.y;
    result.axis.z = vectorA.axis.z + vectorB.axis.z;
    return result;
}

/**
 * @brief Returns vector B subtracted from vector A.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Vector B subtracted from vector A.
 */
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
    FusionVector result;
    result.axis.x = vectorA.axis.x - vectorB.axis.x;
    result.axis.y = vectorA.axis.y - vectorB.axis.y;
    result.axis.z = vectorA.axis.z - vectorB.axis.z;
    return result;
}

/**
 * @brief Returns the sum of the elements.
 * @param vector Vector.
 * @return Sum of the elements.
 */
static inline fixed_pt_num FusionVectorSum(const FusionVector vector) {
    return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief Returns the multiplication of a vector by a scalar.
 * @param vector Vector.
 * @param scalar Scalar.
 * @return Multiplication of a vector by a scalar.
 */
static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const fixed_pt_num scalar) {
    FusionVector result;
    result.axis.x = vector.axis.x * scalar;
    result.axis.y = vector.axis.y * scalar;
    result.axis.z = vector.axis.z * scalar;
    return result;
}

/**
 * @brief Calculates the Hadamard product (element-wise multiplication).
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Hadamard product.
 */
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
    FusionVector result;
    result.axis.x = vectorA.axis.x * vectorB.axis.x;
    result.axis.y = vectorA.axis.y * vectorB.axis.y;
    result.axis.z = vectorA.axis.z * vectorB.axis.z;
    return result;
}

/**
 * @brief Returns the cross product.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Cross product.
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
#define A vectorA.axis
#define B vectorB.axis
    FusionVector result;
    result.axis.x = A.y * B.z - A.z * B.y;
    result.axis.y = A.z * B.x - A.x * B.z;
    result.axis.z = A.x * B.y - A.y * B.x;
    return result;
#undef A
#undef B
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline fixed_pt_num FusionVectorMagnitudeSquared(const FusionVector vector) {
    return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline fixed_pt_num FusionVectorMagnitude(const FusionVector vector) {
    return sqrt(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the normalised vector.
 * @param vector Vector.
 * @return Normalised vector.
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
#ifdef FUSION_USE_NORMAL_SQRT
    const fixed_pt_num magnitudeReciprocal = fixed_pt_num{1.0} / sqrt(FusionVectorMagnitudeSquared(vector));
#else
    const fixed_pt_num magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
#endif
    return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

//------------------------------------------------------------------------------
// Inline functions - Quaternion operations

/**
 * @brief Returns the sum of two quaternions.
 * @param quaternionA Quaternion A.
 * @param quaternionB Quaternion B.
 * @return Sum of two quaternions.
 */
static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    FusionQuaternion result;
    result.element.w = quaternionA.element.w + quaternionB.element.w;
    result.element.x = quaternionA.element.x + quaternionB.element.x;
    result.element.y = quaternionA.element.y + quaternionB.element.y;
    result.element.z = quaternionA.element.z + quaternionB.element.z;
    return result;
}

/**
 * @brief Returns the multiplication of two quaternions.
 * @param quaternionA Quaternion A (to be post-multiplied).
 * @param quaternionB Quaternion B (to be pre-multiplied).
 * @return Multiplication of two quaternions.
 */
static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
#define A quaternionA.element
#define B quaternionB.element
    FusionQuaternion result;
    result.element.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
    result.element.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
    result.element.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
    result.element.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
    return result;
#undef A
#undef B
}

/**
 * @brief Returns the multiplication of a quaternion with a vector.  This is a
 * normal quaternion multiplication where the vector is treated a
 * quaternion with a W element value of zero.  The quaternion is post-
 * multiplied by the vector.
 * @param quaternion Quaternion.
 * @param vector Vector.
 * @return Multiplication of a quaternion with a vector.
 */
static inline FusionQuaternion FusionQuaternionMultiplyVector(const FusionQuaternion quaternion, const FusionVector vector) {
#define Q quaternion.element
#define V vector.axis
    FusionQuaternion result;
    result.element.w = -Q.x * V.x - Q.y * V.y - Q.z * V.z;
    result.element.x = Q.w * V.x + Q.y * V.z - Q.z * V.y;
    result.element.y = Q.w * V.y - Q.x * V.z + Q.z * V.x;
    result.element.z = Q.w * V.z + Q.x * V.y - Q.y * V.x;
    return result;
#undef Q
#undef V
}

/**
 * @brief Returns the normalised quaternion.
 * @param quaternion Quaternion.
 * @return Normalised quaternion.
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
#define Q quaternion.element
#ifdef FUSION_USE_NORMAL_SQRT
    const fixed_pt_num magnitudeReciprocal = fixed_pt_num{1.0} / sqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#else
    const fixed_pt_num magnitudeReciprocal = FusionFastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#endif
    FusionQuaternion normalisedQuaternion;
    normalisedQuaternion.element.w = Q.w * magnitudeReciprocal;
    normalisedQuaternion.element.x = Q.x * magnitudeReciprocal;
    normalisedQuaternion.element.y = Q.y * magnitudeReciprocal;
    normalisedQuaternion.element.z = Q.z * magnitudeReciprocal;
    return normalisedQuaternion;
#undef Q
}

//------------------------------------------------------------------------------
// Inline functions - Matrix operations

/**
 * @brief Returns the multiplication of a matrix with a vector.
 * @param matrix Matrix.
 * @param vector Vector.
 * @return Multiplication of a matrix with a vector.
 */
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
#define R matrix.element
    FusionVector result;
    result.axis.x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z;
    result.axis.y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z;
    result.axis.z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z;
    return result;
#undef R
}

//------------------------------------------------------------------------------
// Inline functions - Conversion operations

/**
 * @brief Converts a quaternion to a rotation matrix.
 * @param quaternion Quaternion.
 * @return Rotation matrix.
 */
static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion quaternion) {
#define Q quaternion.element
    const fixed_pt_num qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const fixed_pt_num qwqx = Q.w * Q.x;
    const fixed_pt_num qwqy = Q.w * Q.y;
    const fixed_pt_num qwqz = Q.w * Q.z;
    const fixed_pt_num qxqy = Q.x * Q.y;
    const fixed_pt_num qxqz = Q.x * Q.z;
    const fixed_pt_num qyqz = Q.y * Q.z;
    FusionMatrix matrix;
    matrix.element.xx = fixed_pt_num{2.0} * (qwqw - fixed_pt_num{0.5} + Q.x * Q.x);
    matrix.element.xy = fixed_pt_num{2.0} * (qxqy - qwqz);
    matrix.element.xz = fixed_pt_num{2.0} * (qxqz + qwqy);
    matrix.element.yx = fixed_pt_num{2.0} * (qxqy + qwqz);
    matrix.element.yy = fixed_pt_num{2.0} * (qwqw - fixed_pt_num{0.5} + Q.y * Q.y);
    matrix.element.yz = fixed_pt_num{2.0} * (qyqz - qwqx);
    matrix.element.zx = fixed_pt_num{2.0} * (qxqz - qwqy);
    matrix.element.zy = fixed_pt_num{2.0} * (qyqz + qwqx);
    matrix.element.zz = fixed_pt_num{2.0} * (qwqw - fixed_pt_num{0.5} + Q.z * Q.z);
    return matrix;
#undef Q
}

/**
 * @brief Converts a quaternion to ZYX Euler angles in degrees.
 * @param quaternion Quaternion.
 * @return Euler angles in degrees.
 */
static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
#define Q quaternion.element
    const fixed_pt_num halfMinusQySquared = fixed_pt_num{0.5} - Q.y * Q.y; // calculate common terms to avoid repeated operations
    FusionEuler euler;
    euler.angle.roll = FusionRadiansToDegrees(atan2(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x));
    euler.angle.pitch = FusionRadiansToDegrees(FusionAsin(fixed_pt_num{2.0} * (Q.w * Q.y - Q.z * Q.x)));
    euler.angle.yaw = FusionRadiansToDegrees(atan2(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z));
    return euler;
#undef Q
}

#endif

//------------------------------------------------------------------------------
// End of file
