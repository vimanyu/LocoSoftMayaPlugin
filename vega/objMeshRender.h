/*************************************************************************
 *                                                                       *
 * Vega FEM Simulation Library Version 1.1                               *
 *                                                                       *
 * "objMesh" library , Copyright (C) 2007 CMU, 2009 MIT, 2012 USC        *
 * All rights reserved.                                                  *
 *                                                                       *
 * Code authors: Jernej Barbic, Christopher Twigg, Daniel Schroeder      *
 * http://www.jernejbarbic.com/code                                      *
 *                                                                       *
 * Research: Jernej Barbic, Fun Shing Sin, Daniel Schroeder,             *
 *           Doug L. James, Jovan Popovic                                *
 *                                                                       *
 * Funding: National Science Foundation, Link Foundation,                *
 *          Singapore-MIT GAMBIT Game Lab,                               *
 *          Zumberge Research and Innovation Fund at USC                 *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the BSD-style license that is            *
 * included with this library in the file LICENSE.txt                    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

// Renders the obj mesh.
// Written by Daniel Schroeder and Jernej Barbic, 2011

#ifndef _OBJMESHRENDER_H_
#define _OBJMESHRENDER_H_

#ifdef WIN32
  #include <windows.h>
#endif

#include "openGL-headers.h"
#include <vector>
#include <assert.h>
#include "objMesh.h"

//flags for ObjMeshRender: 
//geometry mode
#define OBJMESHRENDER_TRIANGLES (1 << 0)
#define OBJMESHRENDER_EDGES (1 << 1)
#define OBJMESHRENDER_VERTICES (1 << 2)

//rendering mode
#define OBJMESHRENDER_NONE           (0)            /* render with only vertices */
#define OBJMESHRENDER_FLAT           (1 << 0)       /* render with facet normals */
#define OBJMESHRENDER_SMOOTH         (1 << 1)       /* render with vertex normals */
#define OBJMESHRENDER_TEXTURE        (1 << 2)       /* render with texture coords */
#define OBJMESHRENDER_COLOR          (1 << 3)       /* render with color materials */
#define OBJMESHRENDER_MATERIAL       (1 << 4)       /* render with materials */
#define OBJMESHRENDER_SELECTION      (1 << 5)       /* render with OpenGL selection (only applies to vertices, otherwise ignored) */
#define OBJMESHRENDER_CUSTOMCOLOR    (1 << 6)       /* render with custom color (only applies to faces)*/

//texture mode
#define OBJMESHRENDER_LIGHTINGMODULATIONBIT 1
#define OBJMESHRENDER_GL_REPLACE 0
#define OBJMESHRENDER_GL_MODULATE 1

#define OBJMESHRENDER_MIPMAPBIT 2
#define OBJMESHRENDER_GL_NOMIPMAP 0
#define OBJMESHRENDER_GL_USEMIPMAP 2

class ObjMeshRender
{
public:

  class Texture
  {
  public:
    Texture() : texture(std::make_pair(false, 0)), textureMode(OBJMESHRENDER_GL_NOMIPMAP | OBJMESHRENDER_GL_MODULATE) {}
    virtual ~Texture() { if(texture.first) glDeleteTextures(1, &(texture.second)); }

    void loadTexture(std::string fullPath, int textureMode);

    bool hasTexture() { return texture.first; }
    unsigned int getTexture() { assert( texture.first ); return texture.second; }

    int getTextureMode() { return textureMode; }

  protected:
    std::pair< bool, unsigned int > texture; // OpenGL texture ID
    int textureMode;
  };

  ObjMeshRender(ObjMesh * mesh);
  ~ObjMeshRender();

  void render(int geometryMode, int renderMode);
  unsigned int createDisplayList(int geometryMode, int renderMode);

  // set custom colors, for OBJMESHRENDER_CUSTOMCOLOR mode
  void setCustomColors(Vec3d color); // constant color for each mesh vertex
  void setCustomColors(std::vector<Vec3d> colors); // specific color for every mesh vertex

  void renderSpecifiedVertices(int * specifiedVertices, int numSpecifiedVertices);
  void renderVertex(int index);

  // the hackier, more specific ones for various specific SceneObject functions
  void renderGroupEdges(char * groupName);

  int numTextures();
  Texture * getTextureHandle(int textureIndex);
  void loadTextures(int textureMode);

  // loads an image from a PPM file; returns the pixels, in bottom-to-top order, and image width and height
  static unsigned char * loadPPM(std::string filename, int * width, int * height); // must be P6 type

  void renderNormals(double normalLength);

  // outputs OpenGL code to render faces
  void outputOpenGLRenderCode();

protected:
  ObjMesh * mesh;
  std::vector<Vec3d> customColors;
  std::vector< Texture > textures;
};

#endif

