#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_
#include "Vector3/Vector3.hpp"
/**
* クォータニオンの足し，引き，掛け算などを簡単にできるようになります．
* @author  Gaku MATSUMOTO
* @bref  クォータニオンを使えるクラスです．
*/

class Quaternion{
public:
	/**
		@bref	Quaternionインスタンスを生成します
	*/
	Quaternion(){
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	};

	/**
	*  @bref Vector3クラスからクォータニオンを作ります
	*/
	Quaternion(Vector3 vector){
		Set(vector);
	}

	/**
	*  @bref   クォータニオンを回転軸と回転角度によって初期化します。
	*  @param   vec 回転軸となる３次元ベクトル
	*  @param   angle 回転角 [rad]
	*/
	Quaternion(Vector3 vec, float angle){
		Set(vec, angle);
	}

	/**
	@bref	要素を代入しながら，インスタンスを生成します．
	@param[in]  _w  実部wの初期値
	@param[in]  _x  虚部iの初期値
	@param[in]  _y  虚部jの初期値
	@param[in]  _z  虚部kの初期値
	*/
	Quaternion(float _w, float _x, float _y, float _z){
		w = _w; x = _x; y = _y; z = _z;
	};

public:
	float w;
	float x;
	float y;
	float z;

	
public:

	/**
	@bref  クォータニオンの要素をコピーします．
	@note  通常の数のように代入できます
	*/
	Quaternion operator=(Quaternion r){
		w = r.w;
		x = r.x;
		y = r.y;
		z = r.z;
		return *this;
	};

	/**
	@bref  クォータニオンを足して代入します．
	@note  通常の数のように代入できます
	*/
	Quaternion operator+=(Quaternion r){
		w += r.w;
		x += r.x;
		y += r.y;
		z += r.z;
		return *this;
	};

	/**
	@bref  クォータニオンを引いて代入します．
	@note  通常の数のように代入できます
	*/
	Quaternion operator-=(Quaternion r){
		w -= r.w;
		x -= r.x;
		y -= r.y;
		z -= r.z;
		return *this;
	};

	/**
	*	@bref	クォータニオンの掛け算をします．
	*	@note	この際も順序は重要です．
	*/
	Quaternion operator*=(Quaternion r){
		static Quaternion QQ;
		QQ.w = w*r.w - x*r.x - y*r.y - z*r.z;
		QQ.x = x*r.w + w*r.x - z*r.y + y*r.z;
		QQ.y = y*r.w + z*r.x + w*r.y - x*r.z;
		QQ.z = z*r.w - y*r.x + x*r.y + w*r.z;
		w = QQ.w;
		x = QQ.x;
		y = QQ.y;
		z = QQ.z;
		return *this;
	};

	/**
		@bref	クォータニオンの複素共役を返します．
		@note	本当はアスタリスクが良かったのですが，ポインタと紛らわしいのでマイナスにしました．
	*/
	Quaternion operator-(){
		Quaternion Q;
		Q.w = w;
		Q.x = -x;
		Q.y = -y;
		Q.z = -z;
		return Q;
	};

	/**
		@bref	クォータニオンを正規化して，単位クォータニオンにします．
		@note	掛け算などを行うたびに実行することをお勧めします．
		@note   ただ、クォータニオンの時間微分は正規化してはいけません
	*/
	void Normalize(){
		float norm = sqrt(w*w + x*x + y*y + z*z);
		if (norm != 0.0f){
			w /= norm;
			x /= norm;
			y /= norm;
			z /= norm;
			return;
		}
		else{
			return;
		}
	};

	/**
	*  @bref   クォータニオンを初期化します
	*/
	template <typename T> void Set(T _w, T _x, T _y, T _z);
	/**
	*  @bref   クォータニオンをVector3クラスで初期化します。
	*/
	void Set(Vector3 vec);

	/**
	*  @bref   クォータニオンを回転軸と回転角度によって初期化します。
	*  param   vec 回転軸となる３次元ベクトル
	*  param   angle 回転角 [rad]
	*/
	void Set(Vector3 vec, float angle){
		vec.Normalize();
		float halfAngle = 0.5f * angle ;
		
		w = cosf(halfAngle);
		x = vec.x * sinf(halfAngle);
		y = vec.y * sinf(halfAngle);
		z = vec.z * sinf(halfAngle);
	}

	/**
	*  @bref   クォータニオンの各要素に配列のようにアクセスします
	*/
	float q(int i){
		float ans = 0.0;
		switch (i){
		case 1:
			ans = w;
			break;
		case 2:
			ans = x;
			break;
		case 3:
			ans = y;
			break;
		case 4:
			ans = z;
			break;
		}
		return ans;
	}

	/**
	*  @bref   クォータニオンのノルムを計算します
	*/
	float Norm(){
		return fabsf(w*w + x*x + y*y + z*z);
	}


	/** クォータニオンとクォータニオンを比較して等しければtrue 等しくなければfalse*/
	bool operator==(Quaternion Q){
		if (w == Q.w && x == Q.x && y == Q.y && z == Q.z){
			return true;
		}
		return false;
	}
	/** クォータニオンとクォータニオンを比較して等しくなければtrue 等しければfalse*/
	bool operator!=(Quaternion Q){
		if (w == Q.w && x == Q.x && y == Q.y && z == Q.z){
			return false;
		}
		return true;
	}
	
	/**
	*  @bref  2つの３次元ベクトルを一致させるクォータニオンを計算
	*  @param from 始点となるベクトルのインスタンス
	*  @param to   終点となるベクトルのインスタンス
	*/
	void FromToRotation(Vector3 from, Vector3 to);

	/**
	@bref   オイラー角で姿勢を取得します．
	@param  val ロール，ピッチ，ヨーの順に配列に格納します．３つ以上の要素の配列を入れてください．
	@note   値は[rad]です．[degree]に変換が必要な場合は別途計算して下さい．
	*/
	void GetEulerAngle(float *val){
		float q0q0 = w * w, q1q1q2q2 = x * x - y * y, q3q3 = z * z;
		val[0] = (atan2f(2.0f * (w * x + y * z), q0q0 - q1q1q2q2 + q3q3));
		val[1] = (-asinf(2.0f * (x * z - w * y)));
		val[2] = (atan2f(2.0f * (x * y + w * z), q0q0 + q1q1q2q2 - q3q3));
	}

	/**
	@bref   オイラー角で姿勢を取得します．
	@param  val ロール，ピッチ，ヨーの順に配列に格納します．３つ以上の要素の配列を入れてください．
	@note   値は[rad]です．[degree]に変換が必要な場合は別途計算して下さい．
	*/
	void GetEulerAngle(Vector3 *v) {
		float q0q0 = w * w, q1q1q2q2 = x * x - y * y, q3q3 = z * z;
		v->x = (atan2f(2.0f * (w * x + y * z), q0q0 - q1q1q2q2 + q3q3));
		v->y = (-asinf(2.0f * (x * z - w * y)));
		v->z = (atan2f(2.0f * (x * y + w * z), q0q0 + q1q1q2q2 - q3q3));
	}

	/**
	*  @bref  クォータニオンをVector3クラスに変換します
	*  @note  クォータニオンのx,y,z成分を持ったベクトルを作ります
	*/
	Vector3 ToVector3(){
		Vector3 vec3(x, y, z);
		return vec3;	
	}

	/**
	*  @bref  3次元ベクトルを回転します
	*  @param v 回転させたい3次元ベクトルのポインタ
	*  @note  余計なオブジェクトを作りません
	*/
	void Rotation(Vector3* v) {
		if (v == NULL) return;
		static float ww = 0.0f;
		static float xx = 0.0f;
		static float yy = 0.0f;
		static float zz = 0.0f;

		static float vx = 0.0f, vy = 0.0f, vz = 0.0f;
		static float _wx, _wy, _wz, _xy, _zx, _yz;
		ww = w * w;
		xx = x * x;
		yy = y * y;
		zz = z * z;

		_wx = w * x;
		_wy = w * y;
		_wz = w * z;
		_xy = x * y;
		_zx = z * x;
		_yz = y * z;

		vx = (ww + xx - yy - zz) * v->x + 2.0f*(_xy - _wz)*v->y + 2.0f*(_zx + _wy) * v->z;
		vy = 2.0f * (_xy + _wz) * v->x + (ww - xx + yy - zz) * v->y + 2.0f*(_yz - _wx)*v->z;
		vz = 2.0f * (_zx - _wy) * v->x + 2.0f * (_wx + _yz)*v->y + (ww - xx - yy + zz)*v->z;

		v->x = vx;
		v->y = vy;
		v->z = vz;
	}

	/**
	*  @bref  3次元ベクトルを回転します.ただし逆回転です
	*  @param v 回転させたい3次元ベクトルのポインタ
	*  @note  余計なオブジェクトを作りません
	*/
	void InvRotation(Vector3* v) {
		if (v == NULL) return;
		static float ww = 0.0f;
		static float xx = 0.0f;
		static float yy = 0.0f;
		static float zz = 0.0f;

		static float vx = 0.0f, vy = 0.0f, vz = 0.0f;
		static float _wx, _wy, _wz, _xy, _xz, _yz;
		ww = w * w;
		xx = x * x;
		yy = y * y;
		zz = z * z;

		_wx = w * x;
		_wy = w * y;
		_wz = w * z;
		_xy = x * y;
		_xz = x * z;
		_yz = y * z;

		vx = (ww + xx - yy - zz) * v->x + 2.0f*(_xy + _wz)*v->y + 2.0f*(_xz - _wy) * v->z;
		vy = 2.0f * (_xy - _wz) * v->x + (ww - xx + yy - zz) * v->y + 2.0f*(_yz + _wx)*v->z;
		vz = 2.0f * (_xz + _wy) * v->x + 2.0f * (-_wx + _yz)*v->y + (ww - xx - yy + zz)*v->z;

		v->x = vx;
		v->y = vy;
		v->z = vz;
	}
};

void Quaternion::FromToRotation(Vector3 from, Vector3 to){
	float halfTheta = 0.5f * from.Angle(to);//回転角度 0からpi/2
	Vector3 axis = from * to;
	axis.Normalize();
	
	w = cos(halfTheta);
	x = axis.x * sin(halfTheta);
	y = axis.y * sin(halfTheta);
	z = axis.z * sin(halfTheta);
}



template<typename T>void Quaternion::Set(T _w, T _x, T _y, T _z){
	w = _w;
	x = _x;
	y = _y;
	z = _z;
	return;
}

void Quaternion::Set(Vector3 vec){
	w = 0.0;
	x = vec.x;
	y = vec.y;
	z = vec.z;
	return;
}

/**
* @fn Quaternion operator*(Quaternion l, Quaternion r)
* @bref クォータニオンの掛け算をします．この際，順序が重要です．
*/
Quaternion operator*(Quaternion l, Quaternion r){
	static Quaternion Q;
	Q.w = l.w*r.w - l.x*r.x - l.y*r.y - l.z*r.z;
	Q.x = l.x*r.w + l.w*r.x - l.z*r.y + l.y*r.z;
	Q.y = l.y*r.w + l.z*r.x + l.w*r.y - l.x*r.z;
	Q.z = l.z*r.w - l.y*r.x + l.x*r.y + l.w*r.z;
	
	return Q;
};

/**
* @fn Quaternion operator*(double s, Quaternion q)
* @bref クォータニオンをスカラー倍します．
*/
Quaternion operator*(float s, Quaternion q){
	static Quaternion Q;
	Q.w = q.w * s;
	Q.x = q.x * s;
	Q.y = q.y * s;
	Q.z = q.z * s;
	return Q;
};

/**
* @fn Quaternion operator*(Quaternion q, double s)
* @bref クォータニオンをスカラー倍します．
*/
Quaternion operator*(Quaternion q, float s){
	static Quaternion Q;
	Q.w = q.w * s;
	Q.x = q.x * s;
	Q.y = q.y * s;
	Q.z = q.z * s;
	return Q;
};

/**
*/
Vector3 operator*(Quaternion q, Vector3 v) {

	static Vector3 ans;
	static float ww = 0.0f;
	static float xx = 0.0f;
	static float yy = 0.0f;
	static float zz = 0.0f;

	//static float vx = 0.0f, vy = 0.0f, vz = 0.0f;
	static float _wx, _wy, _wz, _xy, _zx, _yz;
	ww = q.w * q.w;
	xx = q.x * q.x;
	yy = q.y * q.y;
	zz = q.z * q.z;

	_wx = q.w * q.x;
	_wy = q.w * q.y;
	_wz = q.w * q.z;
	_xy = q.x * q.y;
	_zx = q.z * q.x;
	_yz = q.y * q.z;

	ans.x = (ww + xx - yy - zz) * v.x + 2.0f*(_xy - _wz)*v.y + 2.0f*(_zx + _wy) * v.z;
	ans.y = 2.0f * (_xy + _wz) * v.x + (ww - xx + yy - zz) * v.y + 2.0f*(_yz - _wx)*v.z;
	ans.z = 2.0f * (_zx - _wy) * v.x + 2.0f * (_wx + _yz)*v.y + (ww - xx - yy + zz)*v.z;

	return ans;
}


/**
	@bref	クォータニオンの足し算をします．
*/
Quaternion operator+(Quaternion l, Quaternion r){
	static Quaternion Q;
	Q.w = l.w + r.w;
	Q.x = l.x + r.x;
	Q.y = l.y + r.y;
	Q.z = l.z + r.z;
	return Q;
}

/**
	@bref	クォータニオンの引き算をします．
*/
Quaternion operator-(Quaternion l, Quaternion r){
	static Quaternion Q;
	Q.w = l.w - r.w;
	Q.x = l.x - r.x;
	Q.y = l.y - r.y;
	Q.z = l.z - r.z;
	return Q;
}

#endif