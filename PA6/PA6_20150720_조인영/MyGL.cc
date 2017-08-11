#include "MyGL.h"
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/constants.hpp>
//------------------------------------------------------------------------------
MyGL::MyGL()
    :
    _doLighting( false ),
    _doClipping( false ),
    _doTriangulate( false ),
    _doRasterize( false ) {
}

//------------------------------------------------------------------------------
MyGL::~MyGL()
{}

void normalize(glm::vec4 * v) { // normalize vector
	double div = pow( pow((*v)[0], 2) + pow((*v)[1], 2) + pow((*v)[2], 2), 0.5 );
	for (int i = 0; i < 3; i++) {
		(*v)[i] /= div;
	}
}

//------------------------------------------------------------------------------
bool MyGL::ComputeLighting( vector<GLVertex> &verts ) {
    if( !_doLighting )
        return false;

	// Phong Illumination
    if( light.enabled ) {
		for (int i = 0; i < (int)verts.size(); i++) {
			
			double NL, VR; // ���� N*L, V*R ����
			glm::vec4 E, L, V;
			glm::vec3 N, R;
			
			//definition
			N = verts[i].normal;
			E = { 0,0,1,1 }; // eye position
	
			//calculation
			L = light.position - verts[i].position;				normalize(&L); // �Ի籤 ���⺤��. ����ȭ
			V = E - verts[i].position;							normalize(&V); // �ü�����. ����ȭ
			NL = N[0] * L[0] + N[1] * L[1] + N[2] * L[2];
			
			for (int j = 0; j < 3; j++) {// R = 2*(NL)N - L, �̹� normalized
				R[j] = 2 * (NL) * verts[i].normal[j] - L[j];
			}
			VR = V[0] * R[0] + V[1] * R[1] + V[2] * R[2];
			
			glm::vec4 d = (light.diffuse * material.diffuse); // k_d * I_d
			glm::vec4 s = (light.specular * material.specular); // k_s * I_s

			// ������ �� ���п� diffuse term ���� max(NL,0)��, specular term ���� max(VR,0)^exponent �� ���� ���Ѵ�.
			d *= __max(0, NL);
			s *= pow(__max(0, VR), material.shininess);

			verts[i].color = light.ambient * material.ambient + d + s;  // Phong Illumination Model
		}
    }
    return true;
}

double interpolation(glm::vec4 v1, glm::vec4 v2, double u, double v, double w, double d) { // interpolation ���(����)�� ����Ѵ�.
	double b = u * v2[0] + v * v2[1] + w * v2[2] + d * v2[3]; // ��� ux + vy + wz + d �� ���� ��� �Ÿ�. v2[3] != 1 �̸� clipping space. �̶��� ������ ������ hyperplane�̶� ���� ����.
	double a = u * v1[0] + v * v1[1] + w * v1[2] + d * v1[3];
	return -a / (b - a);
}

void ClipLine( const vector<GLVertex> &vertsIn, vector<GLVertex> &vertsOut, double u, double v, double w, double d ) {
	// suppose: a line segment vector can be represented by : v1 -> v2
	int size = vertsIn.size();

	if (size == 1) {
		vertsOut = vertsIn;
		return;
	}

	for (int i = 0; i < size; i++) { // Sutherland-Hodgman Algorithm. There are four cases between one line segment and one clipping plane (or line).
		glm::vec4 v1 = vertsIn[i].position;
		glm::vec4 v2 = vertsIn[(i + 1) % size].position; // % size: for cyclical accessing, and avoiding [index out of bounds] exception
		bool v1In, v2In;
		v1In = (u*v1[0] + v*v1[1] + w*v1[2] + d*v1[3] >= 0) ? true : false; // inside(or on) or outside
		v2In = (u*v2[0] + v*v2[1] + w*v2[2] + d*v2[3] >= 0) ? true : false; // inside(or on) or outside

		if (!v1In && v2In) { //case 1: v1: out, v2: in => output: v1', v2          where v1' is interpolated vertex from v1 and v2
			double k = interpolation(v1, v2, u, v, w, d); // compute interpolation coefficient k
			GLVertex newVert;
			for (int j = 0; j < 4; j++)	newVert.position[j] = v1[j] + k * (v2[j] - v1[j]); // linear interpolating for position
			for (int j = 0; j < 4; j++)	newVert.color[j] = vertsIn[i].color[j] + k * (vertsIn[(i + 1) % size].color[j] - vertsIn[i].color[j]); // linear interpolating for color
			for (int j = 0; j < 3; j++)	newVert.normal[j] = vertsIn[i].normal[j] + k * (vertsIn[(i + 1) % size].normal[j] - vertsIn[i].normal[j]); // linear interpolating for normal vector
			for (int j = 0; j < 2; j++)	newVert.texCoord[j] = vertsIn[i].texCoord[j] + k * (vertsIn[(i + 1) % size].texCoord[j] - vertsIn[i].texCoord[j]); // linear interpolating for texture coordinates
			vertsOut.push_back(newVert);
			vertsOut.push_back(vertsIn[(i + 1) % size]);
		}
		else if (v1In && v2In) { //case 2: v1: in, v2: in -> output: v2
			//if (i == 0) vertsOut.push_back(vertsIn[i]); // if v1 is the first vertex of the polygon, push v1
			vertsOut.push_back(vertsIn[(i + 1) % size]);
		}
		else if (v1In && !v2In) { //case 3: v1: in, v2: out -> output: v1'
			double k = interpolation(v1, v2, u, v, w, d); // compute interpolation coefficient k
			GLVertex newVert;
			for (int j = 0; j < 4; j++)	newVert.position[j] = v1[j] + k * (v2[j] - v1[j]);
			for (int j = 0; j < 4; j++)	newVert.color[j] = vertsIn[i].color[j] + k * (vertsIn[(i + 1) % size].color[j] - vertsIn[i].color[j]);
			for (int j = 0; j < 3; j++)	newVert.normal[j] = vertsIn[i].normal[j] + k * (vertsIn[(i + 1) % size].normal[j] - vertsIn[i].normal[j]);
			for (int j = 0; j < 2; j++)	newVert.texCoord[j] = vertsIn[i].texCoord[j] + k * (vertsIn[(i + 1) % size].texCoord[j] - vertsIn[i].texCoord[j]);
			//if (i == 0) vertsOut.push_back(vertsIn[i]); // if v1 is the first vertex of the polygon, push v1
			vertsOut.push_back(newVert);
		} //case 4: v1: out, v2: out -> output: none
	}

}

//------------------------------------------------------------------------------
bool MyGL::ClipPolygon( const vector<GLVertex> &vertsIn, vector<GLVertex> &vertsOut ) {
    if( !_doClipping )
        return false;

	vector<GLVertex> cv1, cv2, cv3; //temporary vector
	
	ClipLine(vertsIn, cv1, -1, 0, 0, 1); // ���⼭ �ڸ� ����
	ClipLine(cv1, cv2, 1, 0, 0, 1); // ���⼭ �� �ڸ���, �ٽ� �װ�
	ClipLine(cv2, cv3, 0, -1, 0, 1); // ���⼭ �� �ڸ���,...
	ClipLine(cv3, vertsOut, 0, 1, 0, 1);
	

    return true;
}

//------------------------------------------------------------------------------
bool MyGL::TriangulatePolygon( const vector<GLVertex> &polygonVerts, vector<GLVertex> &triangleVerts ) {
    if( !_doTriangulate )
        return false;

    if( polygonVerts.size() >= 3 ) {
		int size = polygonVerts.size();
		for (int i = 1; i <= size - 2; i++) { // �׳� ù ���ؽ��� ���� �������̶� �����ϰ� ���ʴ�� triangleVerts�� ����ִ´�.
			triangleVerts.push_back(polygonVerts[0]); 
			triangleVerts.push_back(polygonVerts[i]);
			triangleVerts.push_back(polygonVerts[i + 1]);
		}
        return true;

    } else {
        return false;
    }
}

bool insideOut(double x, double y, double x0, double y0, double x1, double y1) { // ������ �������� �ܺ����� �Ǻ��Ѵ�. �� ��Ȯ�� ���ϸ� ĥ�ص� �Ǵ��� �ȵǴ���.
	double A = y0 - y1;
	double B = x1 - x0;
	double C = x0 * y1 - x1 * y0;

	bool t = (A != 0) ? (A > 0) : (B > 0); // tie-breaker

	return (A*x + B*y + C > 0) || (A*x + B*y + C == 0 && t);
}

bool insideTriangle(double x, double y, glm::vec4 v0, glm::vec4 v1, glm::vec4 v2) { // �ﰢ���� �������� �ܺ����� �Ǻ�. Edges �鿡�� ���⼺�� �ִ�.
	return (insideOut(x, y, v0[0], v0[1], v1[0], v1[1]) &&
		insideOut(x, y, v1[0], v1[1], v2[0], v2[1]) &&
		insideOut(x, y, v2[0], v2[1], v0[0], v0[1]));
}

// someCalculation: colorZ, colorO, colorT�� �̿��Ͽ� attribute(color���� ����, tex, normal, ...) interpolate�� �ʿ��� ����� ����Ͽ� mulOut�� ����.
void someCalculation(GLVertex verts[3], glm::vec4 colorZ, glm::vec4 colorO, glm::vec4 colorT, glm_vec4 * mulOut) { // ���� ���� ���� ��, RasterizeTriangle ���� ���� �ڵ带 �Լ��� ���� �Ϳ� �Ұ�. �ؽ��� ��ǥ interpolate���� ��밡��.
	// Solving for Linear Interpolation Equations
	// [Ar Br Cr 0]		[r0 r1 r2 0]	[(e0)^t 0]
	// [Ag Bg Cg 0]		[g0 g1 g2 0]	[(e1)^t 0]
	// [Ab Bb Cb 0]  =  [b0 b1 b2 0]  * [(e2)^t 0]  /  (div)    .............. by the Formula on p.22, Lecture08.pdf  (slightly modified)
	// [Aa Ba Ca 0]		[a0 a1 a2 0]	[0 0 0  0]
	// mulOut        =  in1           * in2  ��� ����.


	// in1 ���� ������.
	glm::vec4 v0 = verts[0].position;
	glm::vec4 v1 = verts[1].position;
	glm::vec4 v2 = verts[2].position;
	double x0 = v0[0], x1 = v1[0], x2 = v2[0],
		y0 = v0[1], y1 = v1[1], y2 = v2[1],
		z0 = v0[2], z1 = v1[2], z2 = v2[2];

	glm_vec4 initial = { 0,0,0,0 };
	glm_vec4 color0 = { colorZ[0], colorZ[1], colorZ[2], colorZ[3] };
	glm_vec4 color1 = { colorO[0], colorO[1], colorO[2], colorO[3] };
	glm_vec4 color2 = { colorT[0], colorT[1], colorT[2], colorT[3] };

	glm_vec4 inT[4] = { color0,
						color1,
						color2,
						initial };
	
	glm_vec4 in1[4];
	glm_mat4_transpose(inT, in1);

	double div = (x1*y2 - x2*y1) - (x0*y2 - x2*y0) + (x0*y1 - x1*y0);

	// in2 �� �������.
	glm_vec4 in2[4] = { { y1 - y2, x2 - x1, x1*y2 - x2*y1, 0 },
	{ y2 - y0, x0 - x2, x2*y0 - x0*y2, 0 },
	{ y0 - y1, x1 - x0, x0*y1 - x1*y0, 0 },
		initial,
	};

	// mulOut�� �������.
	glm_mat4_mul(in2, in1, mulOut);
}


//------------------------------------------------------------------------------
bool MyGL::RasterizeTriangle( GLVertex verts[3] ) {
    if( !_doRasterize )
        return false;

	glm::vec4 v0 = verts[0].position;
	glm::vec4 v1 = verts[1].position;
	glm::vec4 v2 = verts[2].position;

	double x0 = v0[0], x1 = v1[0], x2 = v2[0], 
		y0 = v0[1], y1 = v1[1], y2 = v2[1],
		z0 = v0[2], z1 = v1[2], z2 = v2[2];


	// for Z-buffering, evaluate the coefficients of the linear interpolation w.r.t z (a.k.a plane equation)
	double det = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
	double A = (z1 - z0) * (y2 - y0) - (z2 - z0) * (y1 - y0); A /= det;
	double B = - (z1 - z0) * (x2 - x0) + (z2 - z0) * (x1 - x0); B /= det;
	double z; // z-value


	double x_max = __max(__max(x0, x1), x2); // variables for computing total bounding box vertex.  �̰��� �ﰢ�� v0v1v2�� ���ϴ� ���簢���̴�.
	double y_max = __max(__max(y0, y1), y2);
	double x_min = __min(__min(x0, x1), x2);
	double y_min = __min(__min(y0, y1), y2);

	double div = (x1*y2 - x2*y1) - (x0*y2 - x2*y0) + (x0*y1 - x1*y0);

	glm_vec4 mulOut[4];
	glm_vec4 texMulOut[4];
	glm::vec4 texCoordZ = { verts[0].texCoord[0], verts[0].texCoord[1] ,0 ,0 };
	glm::vec4 texCoordO = { verts[1].texCoord[0], verts[1].texCoord[1] ,0 ,0 };
	glm::vec4 texCoordT = { verts[2].texCoord[0], verts[2].texCoord[1] ,0 ,0 };

	someCalculation(verts, verts[0].color, verts[1].color, verts[2].color, mulOut); // mulOut �� color�� interpolation ������� ����ؼ� �ִ´�.
	someCalculation(verts, texCoordZ, texCoordO, texCoordT, texMulOut); // texMulOut �� texture�� interpolation ������� ����ؼ� �ִ´�.
	
	glm_vec4 rgba; // glm_vec4 -> glm::vec4���� ��ȯ�� ���� �ӽ������.
	glm_vec4 t; // glm_vec4 -> glm::vec4���� ��ȯ�� ���� �ӽ������.

	glm::vec4 color;
	glm::vec2 texCoord;

	for (int x = x_min; x <= x_max; x++) { // total bounding box�� ����.
		for (int y = y_min; y <= y_max; y++) {
			if (insideTriangle(x, y, v0, v1, v2) || insideTriangle(x, y, v0, v2, v1)) { // �ﰢ�� ���ؽ����� orientation�� ���������� ��.�ܺ� �Ǻ�.
					
				glm_vec4 vecMulIn2 = { x, y, 1, 0 };
				
				// edit
				// color�� �������.
				if (textureEnabled) { // ���� �ؽ��ĸ� ������ �Ѵٸ�,
					if (texture.id != 0) { // texMulOut�� �̿��� interpolate �Ѵ�.
						t = glm_vec4_mul_mat4(vecMulIn2, texMulOut);
						texCoord = { t.m128_f32[0] / div, t.m128_f32[1] / div };  // ����Ϸ�.
						int x = (int)(texCoord[0] * (texture.width - 1) + 0.5); // �Ҽ��� ù° �ڸ� �ݿø����� Nearest Neighbor Filtering ����
						int y = (int)(texCoord[1] * (texture.height - 1) + 0.5);
						color = texture.GetPixel(x, y, 0);
					}
				}
				else {  // �ؽ��İ� �ʿ���ٸ�, PA5�� ����.
					rgba = glm_vec4_mul_mat4(vecMulIn2, mulOut);
					color = { rgba.m128_f32[0] / div, rgba.m128_f32[1] / div,
							  rgba.m128_f32[2] / div, rgba.m128_f32[3] / div };
				}
				


				// interpolating for Z-buffering
				z = z0 + A * (x - x0) + B * (y - y0);

				//z_buffering
				if (depthTestEnabled){ 
					// d�� ������ depthTest�� �ؼ� ���� ���� ����Ѵ�.(���� ���� �����Ѵ�) 
					// ���� Illumination����, �� �������� ����ϴ� ������ depend�ϰ� ���� �����ǹ����� �޸��� �׸��� ���� ��µǴ� ��찡 �߻��Ѵ�.
					// �̶� depthTest�� ���ָ� ������ �ذ�ȴ�.
					if (frameBuffer.GetDepth(x, y) > z) {
						frameBuffer.SetDepth(x, y, z);
						frameBuffer.SetPixel(x, y, color);
					} 
				}
				else if (!depthTestEnabled) {
					frameBuffer.SetPixel(x, y, color);
				}
				//end
			}
		}

	}	

    return true;
}