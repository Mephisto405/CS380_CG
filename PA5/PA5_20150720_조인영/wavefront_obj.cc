#include "wavefront_obj.h"
#include "GLRenderer.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <GL/glut.h>
#include <glm/gtc/type_ptr.hpp>

namespace {

//Given three vertices forming a triangle, compute the normal direction of its face
glm::dvec3 compute_face_normal( glm::dvec3 &v0, glm::dvec3 &v1, glm::dvec3 &v2 ) {
    glm::dvec3 n = glm::cross( v1 - v0, v2 - v0 );
    return glm::normalize( n );
}

//Given a list (represented by a pair of iterators) of vertices, compute axis-aligned bounding box (AABB).
template<class IterT>
std::pair<glm::dvec3, glm::dvec3> compute_aabb( IterT vert_begin, IterT vert_end ) {
    std::pair<glm::dvec3, glm::dvec3> aabb{ glm::dvec3{0, 0, 0}, glm::dvec3{0, 0, 0} };

    for( ; vert_begin != vert_end; ++vert_begin ) {
        auto &vert = *vert_begin;

        for ( int i = 0; i < 3; ++i ) {
            aabb.first[i] = std::min( aabb.first[i], vert[i] );
            aabb.second[i] = std::max( aabb.second[i], vert[i] );
        }
    }

    return aabb;
}

}

wavefront_obj_t::wavefront_obj_t( const std::string &path ) {
    is_flat = true;

    std::string line;
    std::istringstream line_stream, item_stream;

    std::ifstream file( path );
    if ( !file )
        throw std::runtime_error( "Cannot open file." );

    while ( std::getline( file, line ) ) {
        // strip off comments and blank lines
        if ( line[0] == '#' || line.size() == 0 )
            continue;

        // parse line
        line_stream.clear();
        line_stream.str( line );

        std::string mode;
        line_stream >> mode;
        if ( mode == "v" ) {
            double x, y, z;
            line_stream >> x >> y >> z;
            vertices.push_back( glm::dvec3{ x, y, z } );
        } else if ( mode == "vn" ) {
            double x, y, z;
            line_stream >> x >> y >> z;
            normals.push_back( glm::normalize( glm::dvec3{ x, y, z } ) );
        } else if ( mode == "vt" ) {
            double u, v;
            line_stream >> u >> v;
            texcoords.push_back( glm::dvec2{u, v} );
        } else if ( mode == "f" ) {
            face_t face;
            std::memset( &face, 0, sizeof( face_t ) );
            face.idx_begin = vertex_indices.size();

            std::string item;
            while ( line_stream >> item ) {
                item_stream.clear();
                item_stream.str( item );

                int v = 0, t = 0, n = 0;
                do {
                    item_stream >> v;
                    char c;
                    if ( !( bool( item_stream >> c ) && c == '/' ) )
                        continue;
                    if ( !( item_stream >> t ) )
                        item_stream.clear();
                    if ( !( bool( item_stream >> c ) && c == '/' ) )
                        continue;
                    if ( !( item_stream >> n ) )
                        item_stream.clear();
                } while ( false );

                vertex_indices.push_back( v - 1 );
                texcoord_indices.push_back( t - 1 );
                normal_indices.push_back( n - 1 );
                ++face.count;

                if ( n )
                    is_flat = false;
            }
            if ( face.count > 2 ) {
                face.normal = compute_face_normal(
                                  vertices[vertex_indices[face.idx_begin]],
                                  vertices[vertex_indices[face.idx_begin + 1]],
                                  vertices[vertex_indices[face.idx_begin + 2]]
                              );
            }

            faces.push_back( face );
        } else if ( mode == "g" ) { // group
        } else if ( mode == "s" ) { // smoothing group
        } else if ( mode == "u" ) { // material line
        } else {
            std::cerr << "Warning: unsupported Wavefront OBJ option: " << mode << "\n";
        }
    }

    aabb = compute_aabb( std::begin( vertices ), std::end( vertices ) );
}


//------------------------------------------------------------------------------
// OpenGL API 를 사용해서 파일에서 읽어둔 object 를 그린다.
// Draw object which is read from file
void wavefront_obj_t::draw() {

	int ptr[1] = { -1 }; //edit: numRasterized 를 저장. -1이면 F4 상태가 아닌 것.
    for ( std::size_t f = 0; f < faces.size(); f++ ) {
        face_t &face = faces[f];
        glBegin( gl_primitive_mode );
        for ( std::size_t v = 0; v < face.count; v++ ) {
            int vi = face.idx_begin + v;
            int i;
            if ( is_flat ) {
                if ( v == 0 ) {
                    glNormal3dv( glm::value_ptr( face.normal ) );
                }
            } else if ( ( i = normal_indices[vi] ) >= 0 ) {
                glNormal3dv( glm::value_ptr( normals[i] ) );
            }
            if ( ( i = texcoord_indices[vi] ) >= 0 ) {
                glTexCoord2dv( glm::value_ptr( texcoords[i] ) );
            }
            if ( ( i = vertex_indices[vi] ) >= 0 ) {
                glVertex3dv( glm::value_ptr( vertices[i] ) );
            }
        }
		
        glEnd(ptr);
    }

	//edit
	if (ptr[0] != -1) {
		printf("numRasterized = %d\n", ptr[0]);
	}
	else { // 기본으로 0을 출력.
		printf("numRasterized = %d\n", 0);
	}

	ptr[0] = -1;
	//end
}
