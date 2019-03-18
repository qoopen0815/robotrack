#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _VECTOR3_HPP_
#define _VECTOR3_HPP_

class Vector3{
public:
	Vector3(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	};
	Vector3(float _x, float _y, float _z){
		x = _x;
		y = _y;
		z = _z;
	};

public:
	float x;
	float y;
	float z;

	/**
	*  @bref ３次元ベクトルの要素をコピー
	*/
	Vector3 operator=(Vector3 vector){
		x = vector.x;
		y = vector.y;
		z = vector.z;
		return *this;
	};

	/**３次元ベクトルを足して代入する*/
	Vector3 operator+=(Vector3 vec){
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	};

	/** ３次元ベクトルを引いて代入する */
	Vector3 operator-=(Vector3 vec){
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return *this;
	};

	/**３次元ベクトルを外積して代入する*/
	Vector3 operator*=(Vector3 vec){
		float _x = y*vec.z - z*vec.y;
		float _y = z*vec.x - x*vec.z;
		float _z = x*vec.y - y*vec.x;
		x = _x;
		y = _y;
		z = _z;
		return *this;
	}

	Vector3 operator/=(float scalar){
		x /= scalar;
		y /= scalar;
		z /= scalar;
		return *this;
	}

	bool operator==(Vector3 vec){
		if ((x == vec.x) && (y == vec.y) && (z == vec.z)){
			return true;
		}
		return false;
	}
	bool operator!=(Vector3 vec){
		if ((x == vec.x) && (y == vec.y) && (z == vec.z)){
			return false;
		}
		return true;
	}
	/**
	*  @bref Vector3クラスの各要素を初期化します。
	*/
	template<typename T>void Set(T _x, T _y, T _z);

	/**
	*  @bref ２つのVector3クラスのなす角を計算します.
	*  @param 自分とのなす角度を計算するVector3クラスのインスタンス
	*/
	float Angle(Vector3 v);
	
	/**
	*  @bref ２つのVector3クラスのなす角を計算します.
	*  @param 自分とのなす角度を計算するVector3クラスのインスタンス
	*  @return 自分 * 相手となるベクトルを回転軸として符号付きのなす角度です
	*/
	float SgnAngle(Vector3 vec);

	/**
	*  @bref ゼロベクトルかどうか判定します.
	*  @note 0除法を防止するのに使ってください.
	*  @return 1ならゼロベクトル、0ならゼロベクトルではありません.
	*/
	int CheckZero(){
		if (x == 0.0f && y == 0.0f && z == 0.0f){
			return 1;
		}
		return 0;
	}

	/**
	*  @bref 自身のノルムを計算して返します.
	*/
	float Norm();

	/**
	*  @bref  単位ベクトルにします
	*/
	void Normalize(){
		float norm = sqrtf(x*x + y*y + z*z);
		if (norm != 0.0f){
			x /= norm;
			y /= norm;
			z /= norm;
			return;
		}
		else{
			return;
		}
	}

	char* toString() {
		char str[128] = {};
		
		sprintf(str, "(%f,%f,%f)", x, y, z);
		return str;
	}
	
};

/**
*  @bref Vector3クラスの各要素の和を計算します。
*/
inline Vector3 operator+(Vector3 left, Vector3 right){
	static Vector3 vec;
	vec.x = left.x + right.x;
	vec.y = left.y + right.y;
	vec.z = left.z + right.z;
	return vec;
}

/**
* @bref Vector3クラスの各要素の差を計算します。
*/
inline Vector3 operator-(Vector3 left, Vector3 right){
	static Vector3 vec;
	vec.x = left.x - right.x;
	vec.y = left.y - right.y;
	vec.z = left.z - right.z;
	return vec;
}

/**
*  @bref Vector3クラスの外積を計算します.
*  @note 外積ですので順序に注意してください.
*/
inline Vector3 operator*(Vector3 left, Vector3 right){
	static Vector3 vec;
	vec.x = left.y * right.z - left.z * right.y;
	vec.y = left.z * right.x - left.x * right.z;
	vec.z = left.x * right.y - left.y * right.x;
	return vec;
}

/**
*  @bref 内積を計算します
*  @note ドットが使えなかったので％になりました。許してヒヤシンス
*/
inline float operator%(Vector3 left, Vector3 right){
	return (left.x * right.x + left.y * right.y + left.z * right.z);
}

/**
*  @bref Vector3クラスの各要素をスカラー倍します
*/
inline Vector3 operator*(float scalar, Vector3 vec3){
	static Vector3 vec;
	vec.x = scalar * vec3.x;
	vec.y = scalar * vec3.y;
	vec.z = scalar * vec3.z;
	return vec;
}

/**
*  @bref Vector3クラスの各要素をスカラー倍します
*/
inline Vector3 operator*(Vector3 vec3, float scalar){
	static Vector3 vec;
	vec.x = scalar * vec3.x;
	vec.y = scalar * vec3.y;
	vec.z = scalar * vec3.z;
	return vec;
}

/**
*  @bref  Vector3クラスの各要素をスカラーで割ります
*/
inline Vector3 operator/(Vector3 vec3, float scalar){
	static Vector3 vec;
	vec.x = vec3.x / scalar;
	vec.y = vec3.y / scalar;
	vec.z = vec3.z / scalar;
	return vec;
}

template<typename T>void Vector3::Set(T _x, T _y, T _z){
	x = _x;
	y = _y;
	z = _z;
}

inline float Vector3::Angle(Vector3 vec){
	float r = (*this % vec) / (this->Norm() * vec.Norm());
	return acosf(r);
}

inline float Vector3::SgnAngle(Vector3 vec){
	float 	theta = this->Angle(vec);
	vec = *this * vec;
	if(vec.z < 0) theta *= -1.0;
	
	return theta;
}

inline float Vector3::Norm(){
	return sqrtf(x*x + y*y + z*z);
}

#endif