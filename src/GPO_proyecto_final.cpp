/************************  GPO_03 ************************************
ATG, 2022
******************************************************************************/

#include <GpO.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/gtc/type_ptr.hpp>
#include <stb_image.h>
#include <vector>
#include <string>
#include <iostream>

// TAMAÑO y TITULO INICIAL de la VENTANA
int ANCHO = 800, ALTO = 600;  // Tamaño inicial ventana
const char* prac = "OpenGL Model Viewer (GpO)";   // Nombre de la practica (aparecera en el titulo de la ventana).

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////     CODIGO SHADERS 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GLSL(src) "#version 330 core\n" #src
static int current_prog = 0;
static int current_model = 0;
#define M_PI 3.141592

/* Vertex Shader de Phong */
const char* vertex_prog1 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
out vec3 n;
out vec2 UV;
out vec3 v;
flat out int TexIndex;
uniform vec3 campos;
uniform mat4 PV;
uniform mat4 M;

void main() {
    gl_Position = PV * M * vec4(pos, 1);

    mat3 M_adj = mat3(transpose(inverse(M)));
    n = M_adj * normal;

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene);
    v = normalize(campos - pos_scene); // Vector desde vértice a cámara
    UV = uv;
    TexIndex = int(texIndex); // Float -> int para indexar texturas
}
);

// Phong - Fragment Shader
const char* fragment_prog1 = GLSL(
    in vec3 n; // Entrada = iluminación de vértices (interpolados en fragmentos)
in vec3 v;
in vec2 UV;
flat in int TexIndex;
float ilu;
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f);
uniform sampler2D textures[16]; // Array de texturas en lugar de 'unit'

void main() {
    vec3 nn = normalize(n);

    vec3 r = reflect(-luz, nn); // Vector de reflexión

    float spec_intensity = max(dot(r, v), 0.0);
    float spec_component = pow(spec_intensity, 16.0); // Elevación a la potencia e=16

    float difusa = max(dot(luz, nn), 0.0);
    ilu = (0.1 + 0.6 * difusa + 0.3 * spec_component); // 10% Ambiente, 60% Difusa, 30% Comp. Especular

    gl_FragColor = texture(textures[TexIndex], UV);
    gl_FragColor = gl_FragColor * ilu;
}
);

/* Vertex Shader de Bling-Phong */
const char* vertex_prog2 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
out vec3 n;
out vec2 UV;
out vec3 v;
flat out int TexIndex;
uniform vec3 campos; // Posición de la cámara en coordenadas del mundo
uniform mat4 M;
uniform mat4 PV;

void main() {
    gl_Position = PV * M * vec4(pos, 1);

    mat3 M_adj = mat3(transpose(inverse(M)));
    n = M_adj * normal;

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene);
    v = normalize(campos - pos_scene); // Vector desde vértice a cámara
    UV = uv;
    TexIndex = int(texIndex); // Float -> int para indexar texturas
}
);

// Bling-Phong - Fragment Shader
const char* fragment_prog2 = GLSL(
    in vec3 n; // Entrada = iluminación de vértices (interpolados en fragmentos)
in vec3 v;
in vec2 UV;
flat in int TexIndex;
float ilu;
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f);
uniform sampler2D textures[16]; // Array de texturas en lugar de 'unit'
vec3 halfVector;  // Half-angle vector

void main() {
    halfVector = normalize(luz + v);

    vec3 nn = normalize(n);

    float spec_intensity = max(dot(nn, halfVector), 0.0);
    float spec_component = pow(spec_intensity, 16.0); // Elevación a la potencia e=16

    float difusa = dot(luz, nn);
    if (difusa < 0) difusa = 0;
    ilu = (0.1 + 0.6 * difusa + 0.3 * spec_component); // 10% Ambiente, 60% Difusa, 30% Comp. Especular

    gl_FragColor = texture(textures[TexIndex], UV);
    gl_FragColor = gl_FragColor * ilu;
}
);

/* Vertex Shader de Resaltado de Silueta */
const char* vertex_prog3 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
out vec3 n;
out vec2 UV;
out vec3 v;
flat out int TexIndex;
uniform vec3 campos;
uniform mat4 M;
uniform mat4 PV;

void main() {
    gl_Position = PV * M * vec4(pos, 1);
    mat3 M_adj = mat3(transpose(inverse(M)));
    n = M_adj * normal;

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene);
    v = normalize(campos - pos_scene);  // Vector desde vértice a cámara
    UV = uv;
    TexIndex = int(texIndex);  // Float -> int para indexar texturas
}
);

// Resaltado de Silueta - Fragment Shader
const char* fragment_prog3 = GLSL(
    in vec3 n;
in vec3 v;
in vec2 UV;
flat in int TexIndex;
uniform sampler2D textures[16]; // Array de texturas en lugar de 'unit'

void main() {
    vec3 nn = normalize(n);
    float angle = degrees(acos(max(dot(nn, v), 0.0)));  // Calcular ángulo entre normal y vista

    if (angle >= 75.0 && angle <= 100.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);  // Silueta negra
    }
    else {
        gl_FragColor = texture(textures[TexIndex], UV);  // Texturizado con array de texturas
    }
}
);

/* Vertex Shader de Toon Shading */
const char* vertex_prog4 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
out vec3 n;
out vec2 UV;
out vec3 v;
out vec3 pos_scene;
flat out int TexIndex;
uniform vec3 campos;
uniform mat4 M;
uniform mat4 PV;

void main() {
    vec4 worldPosition = M * vec4(pos, 1.0);
    gl_Position = PV * worldPosition;

    mat3 M_adj = mat3(transpose(inverse(M)));
    n = M_adj * normal;

    pos_scene = worldPosition.xyz;
    v = normalize(campos - pos_scene); // Vector desde vértice a cámara
    UV = uv;
    TexIndex = int(texIndex); // Float -> int para indexar texturas
}
);

// Toon Shading - Fragment Shader
const char* fragment_prog4 = GLSL(
    in vec3 n; // Entrada = iluminación de vértices (interpolados en fragmentos)
in vec3 v;
in vec2 UV;
in vec3 pos_scene;
flat in int TexIndex;
float ilu;
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f);
uniform sampler2D textures[16]; // Array de texturas en lugar de 'unit'

void main() {
    vec3 nn = normalize(n);
    vec3 luz_final = normalize(luz - pos_scene);
    vec3 v_final = normalize(v);
    vec3 r = reflect(-luz, nn); // Vector de reflexión

    float spec_intensity = max(dot(r, v_final), 0.0);
    float spec = pow(spec_intensity, 16.0); // Elevación a la potencia e=16
    spec = floor(spec * 3.0) / 3.0;


    float difusa_aux = max(dot(luz, nn), 0.0);
    difusa_aux = floor(difusa_aux * 5.0) / 5.0;
    vec3 baseColor = texture(textures[TexIndex], UV).rgb; // Uso de las texturas
    vec3 ambiente = 0.1 * baseColor;
    vec3 difusa = difusa_aux * baseColor;
    vec3 specular = baseColor * spec;
    gl_FragColor = vec4(ambiente + difusa + specular, 1.0);
}
);

/* Vertex Shader de Stippling */
const char* vertex_prog5 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
out vec3 n;
out vec2 UV;
out vec3 v;
flat out int TexIndex;
uniform vec3 campos; // Posición de la cámara en coordenadas del mundo
uniform mat4 M;
uniform mat4 PV;

void main() {
    vec4 worldPosition = M * vec4(pos, 1.0);
    gl_Position = PV * worldPosition;

    mat3 M_adj = mat3(transpose(inverse(M)));
    n = normalize(M_adj * normal);

    vec3 pos_scene = vec3(worldPosition);
    v = normalize(campos - pos_scene); // Vector desde vértice a cámara
    UV = uv;
    TexIndex = int(texIndex);
}
);

// Stippling - Fragment Shader
const char* fragment_prog5 = GLSL(
    uniform sampler2D textures[16];  // Arreglo de muestreadores de texturas
uniform vec3 luz;  // Dirección de la luz
uniform vec3 lightPos;  // Posición de la luz
uniform float umbral = 1.45; // Umbral para el punteado

in vec3 n;  // Vector normal
in vec3 v;  // Vector de vista
in vec2 UV;  // Coordenadas UV
flat in int TexIndex;  // Índice para el arreglo de texturas
out vec4 col;  // Color de salida

// Matriz de umbrales para el patrón de punteado
const float stipplePattern[16] = float[16](
    0.0, 0.6, 0.3, 0.8,
    1.0, 0.4, 0.9, 0.5,
    0.7, 1.0, 0.2, 0.6,
    0.3, 0.8, 0.4, 0.9
    );

void main() {
    vec3 nn = normalize(n);  // Normalizar el vector normal
    vec3 lightDir = normalize(luz - v);  // Calcular la dirección de la luz

    float diff = max(dot(nn, lightDir), 0.0);  // Componente difusa
    vec3 diffuse = diff * texture(textures[TexIndex], UV).rgb;  // Aplicar textura a la luz difusa

    vec3 viewDir = normalize(-v);  // Dirección de vista
    vec3 reflectDir = reflect(-lightDir, nn);  // Dirección de reflexión
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16.0);  // Componente especular
    vec3 specular = spec * texture(textures[TexIndex], UV).rgb;  // Aplicar textura a la luz especular

    vec3 baseColor = texture(textures[TexIndex], UV).rgb;  // Color base de la textura
    vec3 result = (0.1 * baseColor) + diffuse + specular;  // Combinar color base, difuso y especular

    // Convertir a escala de grises
    float luminance = dot(result, vec3(0.299, 0.587, 0.114));

    // Detección de bordes usando el filtro Sobel
    vec2 texelSize = vec2(1.0) / textureSize(textures[TexIndex], 0);
    float edgeDetection = 0.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).r * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, -1)).r * -2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).r * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).r * 1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, 1)).r * 2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).r * 1.0;

    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).b * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, -1)).b * -2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).b * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).b * 1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, 1)).b * 2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).b * 1.0;

    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).g * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, -1)).g * -2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).g * -1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).g * 1.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(0, 1)).g * 2.0;
    edgeDetection += texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).g * 1.0;

    edgeDetection = 1.0 - edgeDetection / 4;

    // Calcular la variación local de la textura
    float variation = 0.0;
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).r - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).r - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).r - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).r - luminance);

    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).g - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).g - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).g - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).g - luminance);

    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, -1)).b - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, -1)).b - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(-1, 1)).b - luminance);
    variation += abs(texture(textures[TexIndex], UV + texelSize * vec2(1, 1)).b - luminance);
    variation /= (4.0 * 3); // Promedio de la variación

    // Ajustar el umbral del patrón de punteado basado en la variación
    float adjustedThreshold = mix(umbral, 0.5, variation);

    // Aplicar el patrón de punteado basado en la detección de bordes y la variación local
    int x = int(mod(gl_FragCoord.x, 4.0));
    int y = int(mod(gl_FragCoord.y, 4.0));
    int index = x + y * 4;
    float threshold = stipplePattern[index];

    if (edgeDetection < threshold * adjustedThreshold) {
        col = vec4(0.0, 0.0, 0.0, 1.0); // Negro
    }
    else {
        col = vec4(1.0, 1.0, 1.0, 1.0); // Blanco
    }
}
);


/* Vertex Shader de Painterly y Pixel Rendering */
const char* vertex_prog6 = GLSL(
    layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in float texIndex;
flat out int TexIndex;
out vec2 UV;
uniform vec3 campos; // Posición de la cámara en coordenadas del mundo
uniform mat4 M;
uniform mat4 PV;

void main()
{
    TexIndex = int(texIndex);
    UV = uv;
    vec4 worldPosition = M * vec4(pos, 1.0);
    gl_Position = PV * worldPosition;
}
);

// Painterly Rendering - Fragment Shader
const char* fragment_prog6 = GLSL(
    flat in int TexIndex; // Índice de la textura actual.
in vec2 UV; // Coordenadas de textura.
uniform sampler2D textures[16];
uniform int brushSize; // Tamaño del pincel.

vec3 coloresCuadrante[4]; // Array para acumular los colores promediados.
vec3 cuadradoCuadrantes[4]; // Array para acumular los cuadrados de los colores.

// Función para acumular colores en los cuadrantes
void cuadrante_pintar(in vec2 UV, in vec2 texSize, int brushSize) {
    for (int i = -brushSize; i <= brushSize; i++) {
        for (int j = -brushSize; j <= brushSize; j++) {
            vec3 texColor = texture2D(textures[TexIndex], UV + vec2(i, j) / texSize).rgb;
            int quadrant = (i <= 0 ? 0 : 2) + (j <= 0 ? 0 : 1);
            coloresCuadrante[quadrant] += texColor;
            cuadradoCuadrantes[quadrant] += texColor * texColor;
        }
    }
}

void main()
{
    // Tamaño de la textura en píxeles.
    vec2 texSize = textureSize(textures[TexIndex], 0);

    // Tamaño del área de pincel en cuadrado.
    float brushSizeSquare = float((brushSize + 1) * (brushSize + 1));

    // Inicializar los arrays a cero.
    for (int k = 0; k < 4; k++) {
        coloresCuadrante[k] = vec3(0.0);
        cuadradoCuadrantes[k] = vec3(0.0);
    }

    cuadrante_pintar(UV, texSize, brushSize);

    // Variable para almacenar el valor mínimo de la suma de los cuadrados de los colores.
    float minColor = 1.0;

    // Calcular el color promedio y la variación en cada cuadrante
    for (int i = 0; i < 4; i++) {
        coloresCuadrante[i] /= brushSizeSquare;
        cuadradoCuadrantes[i] = abs(cuadradoCuadrantes[i] / brushSizeSquare - coloresCuadrante[i] * coloresCuadrante[i]);

        float sumColorSquare = cuadradoCuadrantes[i].r + cuadradoCuadrantes[i].g + cuadradoCuadrantes[i].b;

        // Determinar el cuadrante con la menor variación de color
        if (sumColorSquare < minColor) {
            minColor = sumColorSquare;
            gl_FragColor = vec4(coloresCuadrante[i], 1.0);
        }
    }
}
);


// Pixel Rendering - Fragment Shader
const char* fragment_prog7 = GLSL(
    flat in int TexIndex;
in vec2 UV;
uniform sampler2D textures[16]; // Array de texturas en lugar de 'unit'

// - Número de pixels en horizontal y vertical
uniform float multiplier = 0.2f;
float Despx = 1 / 100.f;
float Despy = 1 / 100.f;
vec2 newUV;


void main()
{
    vec2 texSize = textureSize(textures[TexIndex], 0);
    Despx = 1.0 / (texSize[0] * multiplier);
    Despy = 1.0 / (texSize[1] * multiplier);

    newUV.x = floor(UV.x / Despx) * Despx;
    newUV.y = floor(UV.y / Despy) * Despy;

    gl_FragColor = texture2D(textures[TexIndex], newUV);
}
);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////   RENDER CODE AND DATA
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<GLuint>> textureIDss;
std::vector<GLuint> textureid;
GLFWwindow* window;
GLuint prog[7];
std::vector<std::vector<Mesh>> meshes;

vec3 campos = vec3(4.0f, 4.0f, 2.0f);
vec3 target = vec3(0.0f, 0.0f, 0.9f);
vec3 up = vec3(0, 0, 1);
float az = 0.0, el = 0.75; // Azimuth y Elevación iniciales 

const std::string modelos[3] = { "./data/Tai Lung/Tailung.obj", "./data/Shrek/Shrek/Shrek.obj", "./data/Buzz/Buzz.obj" };


mat4 PP, VV, M; // Matrices de proyeccion y perspectiva

GLfloat umbral = 1.5; // Umbral para programa de Stippling
GLuint brush = 4;

void init_scene() {
    prog[0] = Compile_Link_Shaders(vertex_prog1, fragment_prog1);
    prog[1] = Compile_Link_Shaders(vertex_prog2, fragment_prog2);
    prog[2] = Compile_Link_Shaders(vertex_prog3, fragment_prog3);
    prog[3] = Compile_Link_Shaders(vertex_prog4, fragment_prog4);
    prog[4] = Compile_Link_Shaders(vertex_prog5, fragment_prog5);
    prog[5] = Compile_Link_Shaders(vertex_prog6, fragment_prog6);
    prog[6] = Compile_Link_Shaders(vertex_prog6, fragment_prog7); // Mismo Vertex Shader que Painterly Rendering
    glUseProgram(prog[0]);

    PP = perspective(glm::radians(70.0f), 4.0f / 3.0f, 0.1f, 20.0f);
    VV = lookAt(campos, target, up);

    for (int i = 0; i < size(modelos); i++) {
        std::vector<Mesh> mesh = cargar_modelo(modelos[i], i);
        meshes.push_back(mesh);
    }
    textureid = getTextures(0);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void update_camera_position(float forward, float strafe, float lift) {
    // Calcular la dirección hacia adelante en el plano XY basada en la posición actual y el objetivo
    vec3 forwardDir = normalize(vec3(target.x - campos.x, target.y - campos.y, 0.0f));
    // Calcular la dirección derecha como el producto cruzado de la dirección hacia adelante y el vector hacia arriba
    vec3 rightDir = normalize(cross(forwardDir, vec3(0.0f, 0.0f, 1.0f)));

    // Actualizar la posición de la cámara basada en los inputs de movimiento
    campos += forward * forwardDir;
    campos += strafe * rightDir;
    campos.z += lift;

    // Mantener la altura del objetivo igual a la altura del modelo
    target.z = campos.z;

    // Recalcular la matriz de vista con la posición actualizada de la cámara y el objetivo
    VV = lookAt(campos, target, up);
    glUniform3fv(glGetUniformLocation(prog[current_prog], "campos"), 1, &campos[0]);
}


void actualizar_iluminacion() {
    // Actualizar iluminación
    glm::vec3 light_dir = glm::vec3(
        cos(el) * cos(az),
        cos(el) * sin(az),
        sin(el));
    glUniform3fv(glGetUniformLocation(prog[current_prog], "luz"), 1, &light_dir[0]);
}

void render_scene() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float t = 0.0f;
    mat4 M1 = rotate(mat4(1.0f), radians(90.0f), vec3(1.0f, 0.0f, 0.0f));
    mat4 M2 = rotate(M1, t / 10.0f, vec3(0.0f, 1.0f, 0.0f));

    transfer_mat4("M", M2);
    transfer_mat4("PV", PP * VV);

    // Bind textures
    for (unsigned int i = 0; i < textureid.size(); i++) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, textureid[i]);
    }

    GLint unitLocation = glGetUniformLocation(prog[current_prog], "textures");
    for (int i = 0; i < textureid.size(); ++i) {
        glUniform1i(unitLocation + i, i);
    }

    // Render meshes
    for (int i = 0; i < meshes[current_model].size(); i++) {
        glBindVertexArray(meshes[current_model][i].VAO);
        glDrawElements(GL_TRIANGLES, meshes[current_model][i].indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// PROGRAMA PRINCIPAL
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    init_GLFW();
    window = Init_Window(prac);
    load_Opengl();
    init_scene();

    while (!glfwWindowShouldClose(window))
    {
        render_scene();
        glfwSwapBuffers(window);
        glfwPollEvents();
        show_info();
    }

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

/////////////////////  FUNCION PARA MOSTRAR INFO EN TITULO DE VENTANA  //////////////
void show_info()
{
    static int fps = 0;
    static double last_tt = 0;
    double elapsed, tt;
    char nombre_ventana[128];

    fps++;
    tt = glfwGetTime();

    elapsed = (tt - last_tt);
    if (elapsed >= 0.5) {
        sprintf_s(nombre_ventana, 128, "%s: %4.0f FPS @ %d x %d", prac, fps / elapsed, ANCHO, ALTO);
        glfwSetWindowTitle(window, nombre_ventana);
        last_tt = tt;
        fps = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////  ASIGNACON FUNCIONES CALLBACK
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Callback de cambio tamaño
void ResizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    ALTO = height;
    ANCHO = width;
}

int aux = 0;
float multiplier = 1.0f;
static void KeyCallback(GLFWwindow* window, int key, int code, int action, int mode)
{
    float forward = 0.0f;
    float strafe = 0.0f;
    float lift = 0.0f;

    switch (key) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, true);
        break;

    case GLFW_KEY_0:
        if (action == GLFW_PRESS) {
            current_model = (current_model + 1) % size(modelos);
            textureid = getTextures(current_model);
        }
        break;

    case GLFW_KEY_1:
        if (action == GLFW_PRESS) {
            current_prog = 0;
            glUseProgram(prog[0]); // Usar el programa correspondiente
            actualizar_iluminacion();
        }
        break;

    case GLFW_KEY_2:
        if (action == GLFW_PRESS) {
            current_prog = 1;
            glUseProgram(prog[1]); // Usar el programa correspondiente
            actualizar_iluminacion();
        }
        break;

    case GLFW_KEY_3:
        if (action == GLFW_PRESS) {
            current_prog = 2;
            glUseProgram(prog[2]); // Usar el programa correspondiente
            update_camera_position(forward, strafe, lift);
            actualizar_iluminacion();
        }
        break;

    case GLFW_KEY_4:
        if (action == GLFW_PRESS) {
            current_prog = 3;
            glUseProgram(prog[3]); // Usar el programa correspondiente
            actualizar_iluminacion();
        }
        break;

    case GLFW_KEY_5:
        if (action == GLFW_PRESS) {
            current_prog = 4;
            glUseProgram(prog[4]); // Usar el programa correspondiente
            actualizar_iluminacion();
            // Mandar umbral para cargar puntos iniciales
            glUniform1f(glGetUniformLocation(prog[4], "umbral"), umbral);
        }
        break;

    case GLFW_KEY_6:
        if (action == GLFW_PRESS) {
            current_prog = 5;
            glUseProgram(prog[5]); // Usar el programa correspondiente
            actualizar_iluminacion();
            glUniform1i(glGetUniformLocation(prog[5], "brushSize"), brush);
        }
        break;

    case GLFW_KEY_7:
        if (action == GLFW_PRESS) {
            current_prog = 6;
            glUseProgram(prog[6]); // Usar el programa correspondiente
            actualizar_iluminacion();
        }
        break;

    case GLFW_KEY_UP:
        if (el < M_PI / 2) {
            el += 0.02;
            actualizar_iluminacion();
        }
        break;
    case GLFW_KEY_DOWN:
        if (el > -M_PI / 2) {
            el -= 0.02;
            actualizar_iluminacion();
        }
        break;
    case GLFW_KEY_LEFT:
        az -= 0.02; // Decrementar azimuth
        actualizar_iluminacion();
        break;
    case GLFW_KEY_RIGHT:
        az += 0.02; // Incrementar azimuth
        actualizar_iluminacion();
        break;

    case GLFW_KEY_W:
        forward += 0.25f;  // Mover hacia adelante
        break;
    case GLFW_KEY_S:
        forward -= 0.25f;  // Mover hacia atrás
        break;
    case GLFW_KEY_A:
        strafe -= 0.25f;  // Mover hacia la izquierda
        break;
    case GLFW_KEY_D:
        strafe += 0.25f;  // Mover hacia la derecha
        break;
    case GLFW_KEY_SPACE:
        lift += 0.25f;  // Mover hacia arriba
        break;
    case GLFW_KEY_LEFT_SHIFT:
        lift -= 0.25f;  // Mover hacia abajo
        break;
    case GLFW_KEY_R:
        campos = vec3(4.0f, 4.0f, 2.0f);  // Reiniciar posición cámara
        break;

    case GLFW_KEY_O: // Aumentar umbral de puntos de Stippling
        if (current_prog == 4 && umbral <= 10) {
            umbral += 0.05;
            glUniform1f(glGetUniformLocation(prog[4], "umbral"), umbral);
        }
        break;
    case GLFW_KEY_L: // Reducir umbral de puntos de Stippling
        if (current_prog == 4 && umbral >= 0) {
            umbral -= 0.05;
            glUniform1f(glGetUniformLocation(prog[4], "umbral"), umbral);
        }
        break;
    case GLFW_KEY_I: // Aumentar tamaño de pincel de Painterly
        if (current_prog == 5 && brush < 20 && aux % 2 == 1) {
            brush++;
            glUniform1i(glGetUniformLocation(prog[5], "brushSize"), brush);
        }
        if (aux == 10000000)
            aux = 0;
        aux++;
        break;
    case GLFW_KEY_K: // Reducir tamaño de pincel de Painterly
        if (current_prog == 5 && brush > 1 && aux % 2 == 1) {
            brush--;
            glUniform1i(glGetUniformLocation(prog[5], "brushSize"), brush);
        }
        if (aux < 0)
            aux = 2;
        aux--;
        break;
    case GLFW_KEY_U: // Reducir tamaño de pincel de Painterly
        if (current_prog == 6 && multiplier > 1 / 64.f && aux % 2 == 1) {
            multiplier *= 3 / 4.f;
            glUniform1f(glGetUniformLocation(prog[6], "multiplier"), multiplier);
        }
        if (aux == 10000000)
            aux = 0;
        aux++;
        break;
    case GLFW_KEY_J: // Reducir tamaño de pincel de Painterly
        if (current_prog == 6 && multiplier <= 1 / 2.f && aux % 2 == 1) {
            multiplier /= 3 / 4.f;
            glUniform1f(glGetUniformLocation(prog[6], "multiplier"), multiplier);
        }
        if (aux < 0)
            aux = 2;
        aux--;
        break;
    }
    update_camera_position(forward, strafe, lift);
}

void asigna_funciones_callback(GLFWwindow* window)
{
    glfwSetWindowSizeCallback(window, ResizeCallback);
    glfwSetKeyCallback(window, KeyCallback);
}
