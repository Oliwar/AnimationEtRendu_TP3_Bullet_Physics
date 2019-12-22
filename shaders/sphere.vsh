attribute vec4 in_position;
attribute vec3 colAttr;
varying vec3 col;

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;

void main() {
    col = colAttr;
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * in_position;
}
