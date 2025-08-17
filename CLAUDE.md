# CLAUDE.md

When I ask you to "show me what each matrix looks like," You should present each relevant matrix or function using explicit mathematical expressions.

You should keep the matrix format clear and well-aligned, such as displaying vectors and matrices in a vertically structured layout.

Here are some examples:

h(X) = [x_{i+1} - x_i - Δt·ẋ_i]
       [y_{i+1} - y_i - Δt·ẏ_i]
       [ẋ_{i+1} - ẋ_i        ]
       [ẏ_{i+1} - ẏ_i        ]

r = z - h(X) = [0] - h(X) = -h(X)
              [0]
              [0]
              [0]


d = √[(x_A - x_B)² + (y_A - y_B)²]
h(X) = max(0, 1 - d/d_safe)

∂h/∂x_A = -1/d_safe · (x_A - x_B)/d
∂h/∂y_A = -1/d_safe · (y_A - y_B)/d  
∂h/∂x_B = 1/d_safe · (x_A - x_B)/d
∂h/∂y_B = 1/d_safe · (y_A - y_B)/d

J = [-1/d_safe · (x_A-x_B)/d,  -1/d_safe · (y_A-y_B)/d,  1/d_safe · (x_A-x_B)/d,  1/d_safe · (y_A-y_B)/d]

When showing the workflow of the code, please use mathematical notation rather than vocabs. Unless the code logic has nothing to do with math (like it's not manipulating a matrix or is just logging...)

Currently, our target is to make all robots moves like one point on a rigid body. otherwise the robot velocities may oppose each other, then when we attach robots to the payload, thay cannot move the payload to the target point.

Currently, only dynamics factor, interrobot factor, obstacle factors are using. Please ignore other factors.

When exploring the project, your highest priority is the .cpp and .h files and config files. We assume other parts are correct.

When adding new function, no need to have try and exception, and no need test the special case with if() because we assume the variables would exist. just add the core functions. unless i told you to add try and exception.

Dont do coding until i tell you to do so.