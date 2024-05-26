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
    layout(location = 0) in vec3 pos; // Posición del vértice
layout(location = 1) in vec3 normal; // Normal del vértice
layout(location = 2) in vec2 uv; // Coordenadas UV del vértice
layout(location = 3) in float texIndex; // Índice de la textura
out vec3 n; // Normal transformada
out vec2 UV; // Coordenadas UV a pasar al fragment shader
out vec3 v; // Vector desde el vértice a la cámara
flat out int TexIndex; // Índice de la textura a pasar al fragment shader
uniform vec3 campos; // Posición de la cámara
uniform mat4 PV; // Matriz de proyección y vista
uniform mat4 M; // Matriz de modelo

void main() {
    gl_Position = PV * M * vec4(pos, 1); // Transformación del vértice

    mat3 M_adj = mat3(transpose(inverse(M))); // Matriz de adjunta de la matriz de modelo
    n = M_adj * normal; // Transformación de la normal

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene); // Posición del vértice en el espacio de la escena
    v = normalize(campos - pos_scene); // Vector desde el vértice a la cámara
    UV = uv; // Pasar coordenadas UV
    TexIndex = int(texIndex); // Convertir índice de textura de float a int
}
);

// Phong - Fragment Shader
const char* fragment_prog1 = GLSL(
    in vec3 n; // Normal interpolada
in vec3 v; // Vector desde el vértice a la cámara interpolado
in vec2 UV; // Coordenadas UV interpoladas
flat in int TexIndex; // Índice de la textura
float ilu; // Factor de iluminación
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f); // Dirección de la luz
uniform sampler2D textures[16]; // Array de texturas

void main() {
    vec3 nn = normalize(n); // Normalizar la normal

    vec3 r = reflect(-luz, nn); // Calcular el vector reflejado

    float spec_intensity = max(dot(r, v), 0.0); // Intensidad especular
    float spec_component = pow(spec_intensity, 16.0); // Componente especular

    float difusa = max(dot(luz, nn), 0.0); // Componente difusa
    ilu = (0.1 + 0.6 * difusa + 0.3 * spec_component); // Suma de componentes de iluminación

    gl_FragColor = texture(textures[TexIndex], UV); // Aplicar textura
    gl_FragColor = gl_FragColor * ilu; // Modificar color por factor de iluminación
}
);


/* Vertex Shader de Bling-Phong */
const char* vertex_prog2 = GLSL(
    layout(location = 0) in vec3 pos; // Posición del vértice
layout(location = 1) in vec3 normal; // Normal del vértice
layout(location = 2) in vec2 uv; // Coordenadas UV del vértice
layout(location = 3) in float texIndex; // Índice de la textura
out vec3 n; // Normal transformada
out vec2 UV; // Coordenadas UV a pasar al fragment shader
out vec3 v; // Vector desde el vértice a la cámara
flat out int TexIndex; // Índice de la textura a pasar al fragment shader
uniform vec3 campos; // Posición de la cámara
uniform mat4 M; // Matriz de modelo
uniform mat4 PV; // Matriz de proyección y vista

void main() {
    gl_Position = PV * M * vec4(pos, 1); // Transformación del vértice

    mat3 M_adj = mat3(transpose(inverse(M))); // Matriz adjunta de la matriz de modelo
    n = M_adj * normal; // Transformación de la normal

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene); // Posición del vértice en el espacio de la escena
    v = normalize(campos - pos_scene); // Vector desde el vértice a la cámara
    UV = uv; // Pasar coordenadas UV
    TexIndex = int(texIndex); // Convertir índice de textura de float a int
}
);

// Bling-Phong - Fragment Shader
const char* fragment_prog2 = GLSL(
    in vec3 n; // Normal interpolada
in vec3 v; // Vector desde el vértice a la cámara interpolado
in vec2 UV; // Coordenadas UV interpoladas
flat in int TexIndex; // Índice de la textura
float ilu; // Factor de iluminación
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f); // Dirección de la luz
uniform sampler2D textures[16]; // Array de texturas
vec3 halfVector;  // Half-angle vector

void main() {
    halfVector = normalize(luz + v); // Calcular el vector de mitad de ángulo

    vec3 nn = normalize(n); // Normalizar la normal

    float spec_intensity = max(dot(nn, halfVector), 0.0); // Intensidad especular
    float spec_component = pow(spec_intensity, 16.0); // Componente especular

    float difusa = dot(luz, nn); // Componente difusa
    if (difusa < 0) difusa = 0;
    ilu = (0.1 + 0.6 * difusa + 0.3 * spec_component); // Suma de componentes de iluminación

    gl_FragColor = texture(textures[TexIndex], UV); // Aplicar textura
    gl_FragColor = gl_FragColor * ilu; // Modificar color por factor de iluminación
}
);


/* Vertex Shader de Resaltado de Silueta */
const char* vertex_prog3 = GLSL(
    layout(location = 0) in vec3 pos; // Posición del vértice
layout(location = 1) in vec3 normal; // Normal del vértice
layout(location = 2) in vec2 uv; // Coordenadas UV del vértice
layout(location = 3) in float texIndex; // Índice de la textura
out vec3 n; // Normal transformada
out vec2 UV; // Coordenadas UV a pasar al fragment shader
out vec3 v; // Vector desde el vértice a la cámara
flat out int TexIndex; // Índice de la textura a pasar al fragment shader
uniform vec3 campos; // Posición de la cámara
uniform mat4 M; // Matriz de modelo
uniform mat4 PV; // Matriz de proyección y vista

void main() {
    gl_Position = PV * M * vec4(pos, 1); // Transformación del vértice
    mat3 M_adj = mat3(transpose(inverse(M))); // Matriz adjunta de la matriz de modelo
    n = M_adj * normal; // Transformación de la normal

    vec4 vertex_position_scene = M * vec4(pos, 1.0);
    vec3 pos_scene = vec3(vertex_position_scene); // Posición del vértice en el espacio de la escena
    v = normalize(campos - pos_scene); // Vector desde el vértice a la cámara
    UV = uv; // Pasar coordenadas UV
    TexIndex = int(texIndex); // Convertir índice de textura de float a int
}
);

// Resaltado de Silueta - Fragment Shader
const char* fragment_prog3 = GLSL(
    in vec3 n; // Normal interpolada
in vec3 v; // Vector desde el vértice a la cámara interpolado
in vec2 UV; // Coordenadas UV interpoladas
flat in int TexIndex; // Índice de la textura
uniform sampler2D textures[16]; // Array de texturas

void main() {
    vec3 nn = normalize(n); // Normalizar la normal
    float angle = degrees(acos(max(dot(nn, v), 0.0))); // Calcular el ángulo entre la normal y la vista

    if (angle >= 75.0 && angle <= 100.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); // Silueta negra
    }
    else {
        gl_FragColor = texture(textures[TexIndex], UV); // Aplicar textura
    }
}
);


/* Vertex Shader de Toon Shading */
const char* vertex_prog4 = GLSL(
    layout(location = 0) in vec3 pos; // Posición del vértice
layout(location = 1) in vec3 normal; // Normal del vértice
layout(location = 2) in vec2 uv; // Coordenadas UV del vértice
layout(location = 3) in float texIndex; // Índice de la textura
out vec3 n; // Normal transformada
out vec2 UV; // Coordenadas UV a pasar al fragment shader
out vec3 pos_scene; // Posición del vértice en el espacio de la escena
out vec3 v; // Vector desde el vértice a la cámara
flat out int TexIndex; // Índice de la textura a pasar al fragment shader
uniform vec3 campos; // Posición de la cámara
uniform mat4 M; // Matriz de modelo
uniform mat4 PV; // Matriz de proyección y vista

void main() {
    vec4 worldPosition = M * vec4(pos, 1.0);
    gl_Position = PV * worldPosition; // Transformación del vértice

    mat3 M_adj = mat3(transpose(inverse(M))); // Matriz adjunta de la matriz de modelo
    n = M_adj * normal; // Transformación de la normal

    pos_scene = worldPosition.xyz; // Posición del vértice en el espacio de la escena
    v = normalize(campos - pos_scene); // Vector desde el vértice a la cámara
    UV = uv; // Pasar coordenadas UV
    TexIndex = int(texIndex); // Convertir índice de textura de float a int
}
);

// Toon Shading - Fragment Shader
const char* fragment_prog4 = GLSL(
    in vec3 n; // Normal interpolada
in vec3 v; // Vector desde el vértice a la cámara interpolado
in vec2 UV; // Coordenadas UV interpoladas
in vec3 pos_scene; // Posición del vértice en el espacio de la escena interpolada
flat in int TexIndex; // Índice de la textura
float ilu; // Factor de iluminación
uniform vec3 luz = vec3(1, 1, 0) / sqrt(2.0f); // Dirección de la luz
uniform sampler2D textures[16]; // Array de texturas

void main() {
    vec3 nn = normalize(n); // Normalizar la normal
    vec3 luz_final = normalize(luz - pos_scene); // Normalizar la luz final
    vec3 v_final = normalize(v); // Normalizar la vista final
    vec3 r = reflect(-luz, nn); // Calcular el vector reflejado

    float spec_intensity = max(dot(r, v_final), 0.0); // Intensidad especular
    float spec = pow(spec_intensity, 16.0); // Componente especular
    spec = floor(spec * 3.0) / 3.0; // Ajuste de la especular

    float difusa_aux = max(dot(luz, nn), 0.0); // Componente difusa auxiliar
    difusa_aux = floor(difusa_aux * 5.0) / 5.0; // Ajuste de la difusa auxiliar
    vec3 baseColor = texture(textures[TexIndex], UV).rgb; // Color base
    vec3 ambiente = 0.1 * baseColor; // Color ambiente
    vec3 difusa = difusa_aux * baseColor; // Color difuso
    vec3 specular = baseColor * spec; // Color especular
    gl_FragColor = vec4(ambiente + difusa + specular, 1.0); // Color final
}
);


// Stippling - Fragment Shader
const char* vertex_prog5 = GLSL(
    layout(location = 0) in vec3 pos;               // Posición del vértice
layout(location = 1) in vec3 normal;            // Normal del vértice
layout(location = 2) in vec2 uv;                // Coordenadas de textura
layout(location = 3) in float texIndex;         // Índice de textura
out vec3 n;                                     // Normal del vértice para el fragmento
out vec2 UV;                                    // Coordenadas de textura para el fragmento
out vec3 v;                                     // Vector de vértice a cámara para el fragmento
flat out int TexIndex;                          // Índice de textura para el fragmento (sin interpolación)
uniform vec3 campos;                            // Posición de la cámara en coordenadas del mundo
uniform mat4 M;                                 // Matriz de modelo
uniform mat4 PV;                                // Matriz de proyección y vista

void main() {
    vec4 worldPosition = M * vec4(pos, 1.0);        // Calcula la posición del vértice en el espacio del mundo
    gl_Position = PV * worldPosition;               // Transforma la posición del vértice a coordenadas de clip

    mat3 M_adj = mat3(transpose(inverse(M)));       // Calcula la matriz de la inversa y transpuesta de la matriz de modelo
    n = normalize(M_adj * normal);                  // Calcula la normal del vértice en el espacio del mundo

    vec3 pos_scene = vec3(worldPosition);           // Calcula la posición del vértice en el espacio de la escena
    v = normalize(campos - pos_scene);              // Calcula el vector del vértice a la cámara
    UV = uv;                                        // Pasa las coordenadas de textura al fragmento
    TexIndex = int(texIndex);                       // Pasa el índice de textura al fragmento
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
    // Entrada del vértice: posición
    layout(location = 0) in vec3 pos;
// Entrada del vértice: normal
layout(location = 1) in vec3 normal;
// Entrada del vértice: coordenadas de textura
layout(location = 2) in vec2 uv;
// Entrada del vértice: índice de la textura
layout(location = 3) in float texIndex;

// Salida plana del índice de textura hacia el fragment shader
flat out int TexIndex;
// Salida de las coordenadas de textura hacia el fragment shader
out vec2 UV;

// Uniformes: posición de la cámara y matrices de transformación
uniform vec3 campos; // Posición de la cámara en coordenadas del mundo
uniform mat4 M; // Matriz de modelo
uniform mat4 PV; // Matriz de proyección-vista

void main()
{
    // Convertir el índice de textura a entero y pasarlo al fragment shader
    TexIndex = int(texIndex);
    // Pasar las coordenadas de textura al fragment shader
    UV = uv;
    // Calcular la posición del vértice en el espacio del mundo
    vec4 worldPosition = M * vec4(pos, 1.0);
    // Calcular la posición final del vértice en el espacio de pantalla
    gl_Position = PV * worldPosition;
}
);


// Painterly Rendering - Fragment Shader
const char* fragment_prog6 = GLSL(
    // Entrada plana del índice de textura desde el vertex shader
    flat in int TexIndex; // Índice de la textura actual.
// Entrada de las coordenadas de textura desde el vertex shader
in vec2 UV; // Coordenadas de textura.

// Uniforme: array de texturas
uniform sampler2D textures[16];
// Uniforme: tamaño del pincel
uniform int brushSize; // Tamaño del pincel.

// Arrays para acumular los colores y los cuadrados de los colores
vec3 coloresCuadrante[4]; // Array para acumular los colores promediados.
vec3 cuadradoCuadrantes[4]; // Array para acumular los cuadrados de los colores.

// Función para acumular colores en los cuadrantes
void cuadrante_pintar(in vec2 UV, in vec2 texSize, int brushSize) {
    // Iterar sobre un área cuadrada alrededor del pixel actual
    for (int i = -brushSize; i <= brushSize; i++) {
        for (int j = -brushSize; j <= brushSize; j++) {
            // Obtener el color de la textura en la posición desplazada
            vec3 texColor = texture2D(textures[TexIndex], UV + vec2(i, j) / texSize).rgb;
            // Determinar el cuadrante correspondiente
            int quadrant = (i <= 0 ? 0 : 2) + (j <= 0 ? 0 : 1);
            // Acumular el color y el cuadrado del color en el cuadrante correspondiente
            coloresCuadrante[quadrant] += texColor;
            cuadradoCuadrantes[quadrant] += texColor * texColor;
        }
    }
}

void main()
{
    // Obtener el tamaño de la textura en píxeles
    vec2 texSize = textureSize(textures[TexIndex], 0);

    // Calcular el área del pincel al cuadrado
    float brushSizeSquare = float((brushSize + 1) * (brushSize + 1));

    // Inicializar los arrays a cero
    for (int k = 0; k < 4; k++) {
        coloresCuadrante[k] = vec3(0.0);
        cuadradoCuadrantes[k] = vec3(0.0);
    }

    // Acumular colores y cuadrados de colores en los cuadrantes
    cuadrante_pintar(UV, texSize, brushSize);

    // Variable para almacenar el valor mínimo de la suma de los cuadrados de los colores
    float minColor = 1.0;

    // Calcular el color promedio y la variación en cada cuadrante
    for (int i = 0; i < 4; i++) {
        // Promediar los colores acumulados en el cuadrante
        coloresCuadrante[i] /= brushSizeSquare;
        // Calcular la variación de color en el cuadrante
        cuadradoCuadrantes[i] = abs(cuadradoCuadrantes[i] / brushSizeSquare - coloresCuadrante[i] * coloresCuadrante[i]);

        // Sumar las variaciones de color en los tres canales (r, g, b)
        float sumColorSquare = cuadradoCuadrantes[i].r + cuadradoCuadrantes[i].g + cuadradoCuadrantes[i].b;

        // Determinar el cuadrante con la menor variación de color
        if (sumColorSquare < minColor) {
            minColor = sumColorSquare;
            // Asignar el color del cuadrante con la menor variación al color del fragmento
            gl_FragColor = vec4(coloresCuadrante[i], 1.0);
        }
    }
}
);


// Pixel Rendering - Fragment Shader
const char* fragment_prog7 = GLSL(
    // Entrada plana del índice de textura desde el vertex shader
    flat in int TexIndex;
// Entrada de las coordenadas de textura desde el vertex shader
in vec2 UV;

// Uniforme: array de texturas (hasta 16 texturas)
uniform sampler2D textures[16];

// Uniforme: multiplicador para ajustar el número de píxeles
uniform float multiplier = 0.2f;

// Desplazamiento en x e y para el efecto pixelado
float Despx = 1 / 100.f;
float Despy = 1 / 100.f;

// Nuevas coordenadas de textura después del efecto pixelado
vec2 newUV;

void main()
{
    // Obtener el tamaño de la textura actual en píxeles
    vec2 texSize = textureSize(textures[TexIndex], 0);

    // Calcular el desplazamiento en x e y basado en el tamaño de la textura y el multiplicador
    Despx = 1.0 / (texSize[0] * multiplier);
    Despy = 1.0 / (texSize[1] * multiplier);

    // Ajustar las coordenadas de textura para crear el efecto pixelado
    newUV.x = floor(UV.x / Despx) * Despx;
    newUV.y = floor(UV.y / Despy) * Despy;

    // Asignar el color del fragmento basado en las nuevas coordenadas de textura pixeladas
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
        std::vector<Mesh> mesh = cargar_modelos(modelos[i], i);
        meshes.push_back(mesh);
    }
    textureid = getTextures(0);
    actualizar_iluminacion();
    update_camera_position(0.0f, 0.0f, 0.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void render_scene() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    M = rotate(mat4(1.0f), radians(90.0f), vec3(1.0f, 0.0f, 0.0f));

    transfer_mat4("M", M);
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
