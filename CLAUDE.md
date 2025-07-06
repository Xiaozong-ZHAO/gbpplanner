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

