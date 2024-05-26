#define minGW

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <iostream>


//###include <glad\glad.h> 
#include <glad/glad.h> 

//###include <glfw\glfw3.h>
#include <GLFW/glfw3.h>


// GLM LIB 
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp> 
#include <glm/gtc/matrix_transform.hpp>

// Version OpenGL usada
#define OPENGL_MAJOR 4
#define OPENGL_MINOR 0


#ifdef minGW
  #define fopen_s(fp,fmt,mode)  *(fp)=fopen((fmt),(mode))
  #define sprintf_s snprintf
#endif

#ifndef minGW
 #define M_PI 3.14159265359f
#endif
  

using namespace glm;


// DEclaraciones  de tipos
typedef struct {GLuint VAO; GLuint Ni; GLuint Nv; GLuint Nt; GLuint tipo_indice; } objeto;
typedef struct {
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    GLuint VAO, VBO, EBO;

    void setupMesh() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        // Position attribute
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);

        // Normal attribute
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));

        // UV attribute
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));

        // Texture index attribute
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(8 * sizeof(float)));

        glBindVertexArray(0);
	}
} Mesh;


/// DECLARACIONES DE FUNCIONES AUXILIARES

void init_scene(void);
void show_info();

void vuelca_mat4(mat4 M);

void check_errores_programa(GLuint);
GLuint compilar_shader(const char*, GLuint);
GLuint link_programa(GLuint, GLuint);
GLuint Compile_Link_Shaders(const char*, const char*);
char* leer_codigo_de_fichero(const char*);


GLuint cargar_cube_map(const char *, GLuint);
GLuint cargar_textura(const char *,GLuint);
objeto cargar_modelos(char*);
std::vector<Mesh> cargar_modelos(const std::string& path, int modelIndex);
GLuint cargarTextura(const char* archivo);
std::vector<GLuint> getTextures(int modelIndex);

void transfer_mat4(const char*, mat4);
void transfer_mat3(const char*, mat3);
void transfer_vec4(const char*, vec4);
void transfer_vec3(const char*, vec3);
void transfer_vec2(const char*, vec2);
void transfer_int(const char*, GLuint);
void transfer_float(const char*, GLfloat);


/*
//// Funciones asociadas a eventos
void framebuffer_size_callback(GLFWwindow*, int, int);
static void ScrollCallback(GLFWwindow*, double, double);
static void KeyCallback(GLFWwindow*, int, int, int, int);
static void CursorPosCallback(GLFWwindow*, double, double);
static void MouseCallback(GLFWwindow*, int, int, int);
*/


// FUNCIONES DE INICIALIZACION de OPENGL y VENTANAS
void asigna_funciones_callback(GLFWwindow*);
void init_GLFW(void);
void load_Opengl(void);
GLFWwindow*  Init_Window(const char*);


