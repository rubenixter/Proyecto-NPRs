#include "GpO.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

// Variables globales
extern int ANCHO;
extern int ALTO;

// Funciones inicialización librerias, ventanas, OpenGL
void init_GLFW(void)
{
	if (!glfwInit())
	{
		fprintf(stdout, "No se inicializo libreria GLFW\n");
		exit(EXIT_FAILURE);
	}
	int Major, Minor, Rev;
	glfwGetVersion(&Major, &Minor, &Rev);
	printf("Libreria GLFW (ver. %d.%d.%d) inicializada\n", Major, Minor, Rev);
}

GLFWwindow* Init_Window(const char* nombre)
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, OPENGL_MAJOR);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, OPENGL_MINOR);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(ANCHO, ALTO, nombre, NULL, NULL);
	if (window == NULL)
	{
		fprintf(stdout, "Fallo al crear ventana GLFW con OpenGL context %d.%d\n", ANCHO, ALTO);
		glfwTerminate();
		exit(EXIT_FAILURE);
		return NULL;
	}

	fprintf(stdout, "Ventana GLFW creada con contexto OpenGL %d.%d\n", OPENGL_MAJOR, OPENGL_MINOR);
	glfwMakeContextCurrent(window);

	asigna_funciones_callback(window);

	return window;
}

void load_Opengl(void)
{
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		fprintf(stdout, "Fallo al cargar funciones de OpenGL con GLAD\n");
		exit(EXIT_FAILURE);
	}
	fprintf(stdout, "OpenGL Version: %s\n", glGetString(GL_VERSION));
	glViewport(0, 0, ANCHO, ALTO);

	printf("---------------------------------------------\n");
	return;
}

// CARGA, COMPILACION Y LINKADO DE LOS SHADERS

GLuint compilar_shader(const char* Shader_source, GLuint type)
{
	switch (type)
	{
	case GL_VERTEX_SHADER: printf("Compilando Vertex Shader :: "); break;
	case GL_FRAGMENT_SHADER: printf("Compilando Fragment Shader :: "); break;
	case GL_GEOMETRY_SHADER: printf("Compilando Geometric Shader :: "); break;
	}

	GLuint ShaderID = glCreateShader(type);

	glShaderSource(ShaderID, 1, &Shader_source, NULL);
	glCompileShader(ShaderID);

	GLint Result = GL_FALSE;
	int InfoLogLength; char error[512];

	glGetShaderiv(ShaderID, GL_COMPILE_STATUS, &Result);
	if (Result == GL_TRUE) fprintf(stdout, "Sin errores\n");
	else
	{
		fprintf(stdout, "ERRORES\n");
		glGetShaderiv(ShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength > 512) InfoLogLength = 512;
		glGetShaderInfoLog(ShaderID, InfoLogLength, NULL, error);
		fprintf(stdout, "\n%s\n", error);
	}
	printf("----------------------------------------------\n");
	return ShaderID;
}

void check_errores_programa(GLuint id)
{
	GLint Result = GL_FALSE;
	int InfoLogLength;
	char error[512];

	printf("Resultados del linker (GPU): ");
	glGetProgramiv(id, GL_LINK_STATUS, &Result);
	if (Result == GL_TRUE) fprintf(stdout, "Sin errores\n");
	else
	{
		fprintf(stdout, "ERRORES\n");
		glGetProgramiv(id, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength < 1) InfoLogLength = 1; if (InfoLogLength > 512) InfoLogLength = 512;
		glGetProgramInfoLog(id, InfoLogLength, NULL, error);
		fprintf(stdout, "\n%s\n", error);
	}
	printf("---------------------------------------------\n");
}

GLuint Compile_Link_Shaders(const char* vertexShader_source, const char* fragmentShader_source)
{
	GLuint VertexShaderID = compilar_shader(vertexShader_source, GL_VERTEX_SHADER);
	GLuint FragmentShaderID = compilar_shader(fragmentShader_source, GL_FRAGMENT_SHADER);

	GLuint ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, FragmentShaderID);

	glLinkProgram(ProgramID);
	check_errores_programa(ProgramID);

	glDetachShader(ProgramID, VertexShaderID);  glDeleteShader(VertexShaderID);
	glDetachShader(ProgramID, FragmentShaderID);  glDeleteShader(FragmentShaderID);

	return ProgramID;
}

// AUXILIARES

void transfer_mat4(const char* name, mat4 M)
{
	GLuint loc;
	GLuint prog;

	glGetIntegerv(GL_CURRENT_PROGRAM, (GLint*)&prog);

	loc = glGetUniformLocation(prog, name);
	if (loc == -1) {
		printf("No existe variable llamada %s en el programa activo de la GPU (%d)\n", name, prog);
		glfwTerminate();
	}
	else glUniformMatrix4fv(loc, 1, GL_FALSE, &M[0][0]);
}

// Matriz de arrays de texturas -> Para poder usar más de un modelo
std::vector<std::vector<GLuint>> textures;

GLuint cargarTextura(const char* archivo) {
	int ancho, alto, canales;
	unsigned char* datos = stbi_load(archivo, &ancho, &alto, &canales, 0);
	if (datos) {
		GLuint texturaID;
		glGenTextures(1, &texturaID);
		glBindTexture(GL_TEXTURE_2D, texturaID);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		GLenum formato = GL_RGB;
		if (canales == 4) {
			formato = GL_RGBA;
		}

		glTexImage2D(GL_TEXTURE_2D, 0, formato, ancho, alto, 0, formato, GL_UNSIGNED_BYTE, datos);
		glGenerateMipmap(GL_TEXTURE_2D);

		stbi_image_free(datos);
		return texturaID;
	}
	else {
		std::cerr << "Failed to load texture: " << archivo << std::endl;
		stbi_image_free(datos);
		return 0;
	}
}

std::vector<Mesh> cargar_modelo(const std::string& path, int modelIndex) {
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenSmoothNormals);
	std::vector<Mesh> meshDatas;

	if (scene && scene->mNumMeshes > 0) {
		// Asegurar que hay espacio suficiente en textureIDs para el índice del modelo
		if (textures.size() <= modelIndex) {
			textures.resize(modelIndex + 1);
		}

		for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
			aiMesh* mesh = scene->mMeshes[i];
			Mesh meshData;
			for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
				aiVector3D pos = mesh->mVertices[j];
				aiVector3D normal = mesh->mNormals[j];
				aiVector3D uv = mesh->mTextureCoords[0][j];
				float texIndex = static_cast<float>(mesh->mMaterialIndex);

				meshData.vertices.insert(meshData.vertices.end(), {
					pos.x, pos.y, pos.z,
					normal.x, normal.y, normal.z,
					uv.x, uv.y,
					texIndex
					});
			}
			for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
				aiFace& face = mesh->mFaces[j];
				for (unsigned int k = 0; k < face.mNumIndices; k++) {
					meshData.indices.push_back(face.mIndices[k]);
				}
			}
			// Load textures
			aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
			aiString str;
			if (material->GetTexture(aiTextureType_DIFFUSE, 0, &str) == AI_SUCCESS) {
				std::string texturePath = str.C_Str();
				GLuint textureID = cargarTextura(texturePath.c_str());
				textures[modelIndex].push_back(textureID);
			}
			meshData.setupMesh();
			meshDatas.push_back(meshData);
		}
	}
	else {
		std::cerr << "Error loading model: " << importer.GetErrorString() << std::endl;
	}
	return meshDatas;
}

std::vector<GLuint> getTextures(int modelIndex) {
	if (modelIndex < textures.size()) {
		return textures[modelIndex];
	}
	else {
		return std::vector<GLuint>(); // Retorna un vector vacío si el índice está fuera de rango
	}
}