---
title: Modern OpenGL Rendering Pipeline
sources:
  - { website: "LearnOpenGL", url: "https://learnopengl.com" }
  - { website: "OpenGL Guide", url: "https://open.gl" }
requires:
  - ../math/linear-algebra/matrix-operations.md
  - ../math/transforms/homogeneous-transforms.md
related:
  - ../simulation/sim-loop.md
  - ../simulation/architecture/simulation-engine-patterns.md
---

# Modern OpenGL Rendering Pipeline

Modern OpenGL (3.3+ core profile) replaces the fixed-function pipeline with programmable shaders. All geometry passes through a vertex shader (positioning) and fragment shader (colouring) written in GLSL. This is the rendering backend for 3D engineering visualisation — the ball-balancer plate, ball, and servo arms.

## Core Objects

### VAO / VBO / EBO

| Object | Purpose |
|---|---|
| **VBO** (Vertex Buffer Object) | Stores vertex data (positions, normals, UVs) in GPU memory |
| **VAO** (Vertex Array Object) | Records the configuration of vertex attribute pointers — which VBOs are bound and how they're laid out |
| **EBO** (Element Buffer Object) | Stores vertex indices for indexed drawing, avoiding vertex duplication |

Setup pattern:

```cpp
GLuint VAO, VBO;
glGenVertexArrays(1, &VAO);
glGenBuffers(1, &VBO);
glBindVertexArray(VAO);
glBindBuffer(GL_ARRAY_BUFFER, VBO);
glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
glEnableVertexAttribArray(0);
glBindVertexArray(0);
```

### Shader Program

A shader program links a vertex shader and fragment shader into an executable pipeline. Compiled at runtime from GLSL source strings.

## GLSL Shaders

### Vertex Shader

Runs once per vertex. Transforms position from model-local space to clip space via the MVP matrix:

```glsl
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 MVP;
uniform mat4 model;

out vec3 FragPos;
out vec3 Normal;

void main() {
    gl_Position = MVP * vec4(aPos, 1.0);
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
}
```

### Fragment Shader

Runs once per fragment. Computes final pixel colour using the Phong lighting model:

```glsl
#version 330 core
in vec3 FragPos;
in vec3 Normal;
out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;
uniform vec3 viewPos;

void main() {
    // Ambient
    vec3 ambient = 0.1 * lightColor;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = 0.5 * spec * lightColor;

    vec3 result = (ambient + diffuse + specular) * objectColor;
    FragColor = vec4(result, 1.0);
}
```

## MVP Transform Chain

All vertex positions pass through three successive matrix transformations:

| Matrix | Transform | Purpose |
|---|---|---|
| **Model** | Local → World | Object position, rotation, scale |
| **View** | World → Camera | Camera's inverse transform (lookAt) |
| **Projection** | Camera → Clip | Perspective frustum or orthographic box |

```
MVP = Projection × View × Model
```

The projection matrix creates the perspective effect (distant objects smaller). The view matrix positions the virtual camera. The model matrix places each object in the world. This chain maps directly to the homogeneous transform pipeline (see `homogeneous-transforms.md`).

## Phong Lighting Model

Three additive components computed per fragment:

| Component | Formula | Visual effect |
|---|---|---|
| **Ambient** | I_a × k_a | Constant background fill |
| **Diffuse** | I_l × k_d × max(N·L, 0) | Directional shading from surface orientation |
| **Specular** | I_l × k_s × max(R·V, 0)^n | Highlight from viewer-dependent reflection |

N = surface normal, L = light direction, R = reflection vector, V = view direction, n = shininess exponent.

## Render Loop

```
Each frame:
  1. glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
  2. glUseProgram(shaderProgram)
  3. Upload uniforms (MVP, light position, camera position)
  4. For each object:
     a. Compute model matrix from object pose
     b. Upload MVP = projection * view * model
     c. glBindVertexArray(objectVAO)
     d. glDrawArrays or glDrawElements
  5. glfwSwapBuffers(window)
  6. glfwPollEvents()
```

The render loop runs at display refresh rate (typically 60 Hz), while the physics simulation runs at a fixed rate (e.g., 1 kHz) using the fixed-timestep accumulator pattern. The `alpha` interpolation factor from `FixedTimestep` is used to interpolate between the previous and current physics state for smooth rendering.

## Supporting Libraries

| Library | Role |
|---|---|
| **GLFW** | Window creation, OpenGL context, keyboard/mouse input |
| **GLAD** | Loads OpenGL function pointers at runtime |
| **GLM** | `glm::mat4`, `glm::perspective()`, `glm::lookAt()` |

For Caliburn projects, GLFW and GLAD handle the platform layer. GLM provides the matrix math (or Eigen with manual 4×4 construction — both work). Dear ImGui renders on top of the OpenGL context for debug UI panels.

## WebGL / Emscripten Notes

OpenGL ES 3.0 (WebGL 2.0) requires:
- `#version 300 es` in shaders (not `#version 330 core`)
- Explicit `precision mediump float;` in fragment shaders
- No DSA functions — use legacy bind-to-modify API
- Single-threaded: `emscripten_set_main_loop` replaces the while-loop

## Ball-Balancer Application

The ball-balancer 3D visualisation needs:
- Plate mesh (rectangular prism) with model matrix from servo angles
- Ball mesh (sphere) with model matrix from ball position on plate
- Servo arm meshes showing actuator positions
- Phong lighting for depth perception
- Camera orbiting the plate centre

Each object's model matrix is computed from the simulation state. The plate model matrix encodes the two-axis tilt from servo angles. The ball model matrix encodes its XY position on the plate surface (in the plate's local frame, then transformed to world via the plate's model matrix).
