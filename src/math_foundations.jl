module MathFoundations

export euclidean_distance, is_collision_free

"""
    euclidean_distance(p1::Tuple{Float64, Float64}, p2::Tuple{Float64, Float64})

Calculate the Euclidean distance between two 2D points.
"""
function euclidean_distance(p1, p2)
    return sqrt((p1[1] - p2[1])^2 + (p1[2] - p2[2])^2)
end

"""
    is_collision_free(p1::Tuple{Float64, Float64}, p2::Tuple{Float64, Float64}, obstacles::Array)

High-performance collision check between a line segment and circular obstacles.
"""
function is_collision_free(p1, p2, obstacles)
    dist = euclidean_distance(p1, p2)
    steps = floor(Int, dist / 0.1) + 2
    for i in 0:steps-1
        t = i / (steps - 1)
        px = p1[1] * (1 - t) + p2[1] * t
        py = p1[2] * (1 - t) + p2[2] * t
        for obs in obstacles
            # obs is (x, y, radius)
            if sqrt((px - obs[1])^2 + (py - obs[2])^2) <= obs[3]
                return false
            end
        end
    end
    return true
end

# Example usage for verification
if abspath(PROGRAM_FILE) == @__FILE__
    obstacles = [(2.0, 2.0, 0.8), (6.0, 7.0, 1.2)]
    println("Distance test: ", euclidean_distance((0,0), (3,4)))
    println("Collision test (free): ", is_collision_free((0,0), (1,1), obstacles))
    println("Collision test (hit): ", is_collision_free((0,0), (3,3), obstacles))
end

end
