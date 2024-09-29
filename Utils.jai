calculate_polygon_centroid :: (vertices: [] Vector2) -> Vector2 {
    centroid: Vector2;
    area: float;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        partial_area := cross(vertices[i], vertices[j]);

        area     += partial_area;
        centroid += partial_area * (vertices[i] + vertices[j]);
    }

    area     /= 2.0;
    centroid /= 6.0 * area;

    return centroid;
}

calculate_polygon_inertia :: (vertices: [] Vector2, mass: float) -> float {
    inertia: float;

    centroid      := calculate_polygon_centroid(vertices);
    triangle_mass := mass / vertices.count;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        ci := vertices[i] - centroid;
        cj := vertices[j] - centroid;

        dots := dot(ci, ci) + dot(cj, cj) + dot(ci, cj);

        triangle_inertia := dots * triangle_mass / 6.0;
        inertia += triangle_inertia;
    }

    return inertia;
}

calculate_circle_inertia :: (radius: float, mass: float) -> float {
    return mass * radius * radius / 2.0;
}

is_point_inside_polygon :: (vertices: [] Vector2, point: Vector2) -> bool {
    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        normal := perpendicular(normalize(vertices[i] - vertices[j]));
        depth  := dot(point - vertices[i], normal);

        if depth > 0.0 return false;
    }

    return true;
}

is_point_inside_circle :: (center: Vector2, radius: float, point: Vector2) -> bool {
    return distance(center, point) <= radius;
}

find_closest_edge_from_point :: (vertices: [] Vector2, point: Vector2) -> [2] Vector2 {
    edge: [2] Vector2;

    min_depth := -FLOAT32_MAX;
    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        normal := perpendicular(normalize(vertices[i] - vertices[j]));
        depth  := dot(point - vertices[i], normal);

        if depth > min_depth {
            min_depth = depth;
            edge[0]   = vertices[i];
            edge[1]   = vertices[j];
        }
    }

    return edge;
}

find_closest_point_on_edge :: (edge: [2] Vector2, point: Vector2) -> Vector2 {
    vector        := edge[1] - edge[0];
    ratio_on_line := dot(point - edge[0], vector) / dot(vector, vector);
    ratio_on_edge := clamp(ratio_on_line, 0.0, 1.0);
    return lerp(edge[0], edge[1], ratio_on_edge);
}

find_closest_point_on_vertices :: (vertices: [] Vector2, point: Vector2) -> Vector2 {
    edge := find_closest_edge_from_point(vertices, point);
    return find_closest_point_on_edge(edge, point);
}

SAT_Contact :: struct {
    point:  Vector2;
    depth:  float = FLOAT32_MAX;
    normal: Vector2;
}

find_sat_collision :: (A: [] Vector2, B: [] Vector2) -> SAT_Contact, bool {
    result: SAT_Contact;

    for i: 0..A.count - 1 {
        j := (i + 1) % A.count;

        normal  := perpendicular(normalize(A[i] - A[j]));
        support := find_sat_support(A[i], normal, B);

        // Separating axis found, no collision
        if support.depth <= 0.0 return result, false;

        if result.depth > support.depth {
            result.point  = support.point;
            result.depth  = support.depth;
            result.normal = normal;
        }
    }

    return result, true;
}

SAT_Support :: struct {
    point: Vector2;
    depth: float = -FLOAT32_MAX;
}

find_sat_support :: (point: Vector2, normal: Vector2, vertices: [] Vector2) -> SAT_Support {
    result: SAT_Support;

    for vertices {
        depth := dot(point - it, normal);

        if result.depth < depth {
            result.point = it;
            result.depth = depth;
        }
    }

    return result;
}

#scope_module

perpendicular :: (using v: Vector2) -> Vector2 #expand {
    return .{ -y, x };
}

cross :: (a: Vector2, b: Vector2) -> float #expand {
    return a.x * b.y - b.x * a.y;
}
