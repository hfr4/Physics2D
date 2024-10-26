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

        edge   : Segment = .[ vertices[i], vertices[j] ];
        normal := segment_normal(edge);
        depth  := dot(point - vertices[i], normal);

        if depth > 0.0 return false;
    }

    return true;
}

is_point_inside_circle :: (center: Vector2, radius: float, point: Vector2) -> bool {
    return distance(center, point) <= radius;
}

find_closest_edge_from_point :: (vertices: [] Vector2, point: Vector2) -> Segment {
    result: Segment;

    max_depth := -FLOAT32_MAX;
    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        edge : Segment = .[ vertices[i], vertices[j] ];
        depth := dot(point - vertices[i], segment_normal(edge));

        if depth > max_depth {
            max_depth = depth;
            result    = edge;

        }
    }

    return result;
}

find_edge_pointed_by_direction :: (vertices: [] Vector2, dir: Vector2) -> Segment {
    result: Segment;

    min_proj := FLOAT32_INFINITY;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        edge : Segment = .[ vertices[i], vertices[j] ];
        normal := segment_normal(edge);
        proj   := dot(normal, dir);

        if proj < min_proj {
            min_proj = proj;
            result   = edge;
        }
    }

    return result;
}

find_closest_point_on_edge :: (edge: Segment, point: Vector2) -> Vector2 {
    vector        := edge[1] - edge[0];
    ratio_on_line := dot(point - edge[0], vector) / dot(vector, vector);
    ratio_on_edge := clamp(ratio_on_line, 0.0, 1.0);
    return lerp(edge[0], edge[1], ratio_on_edge);
}

find_closest_point_on_vertices :: (vertices: [] Vector2, point: Vector2) -> Vector2 {
    edge := find_closest_edge_from_point(vertices, point);
    return find_closest_point_on_edge(edge, point);
}

find_smallest_separating_edge_and_depth_using_SAT :: (vertices_a: [] Vector2, vertices_b: [] Vector2) -> Segment, float {
    result: Segment;
    depth := -FLOAT32_INFINITY;

    for i: 0..vertices_a.count - 1 {
        j := (i + 1) % vertices_a.count;

        edge : Segment = .[ vertices_a[i], vertices_a[j] ];
        normal := segment_normal(edge);

        min_proj := FLOAT32_INFINITY;
        for b: vertices_b {
            proj := dot((b - edge[0]), normal);
            if min_proj > proj {
                min_proj = proj;
            }
        }

        if depth < min_proj {
            depth  = min_proj;
            result = edge;
        }
    }

    return result, depth;
}

clip_segment_with_line :: (segment: Segment, line: Segment) -> Segment, int {
    clipped: Segment;
    count: int;

    c0 := cross(segment[0] - line[0], line[1] - line[0]);
    c1 := cross(segment[1] - line[0], line[1] - line[0]);

    if c0 <= 0.0 {
        clipped[count] = segment[0];
        count += 1;
    }

    if c1 <= 0.0 {
        clipped[count] = segment[1];
        count += 1;
    }

    if c0 * c1 < 0.0 {
        clipped[count] = lerp(segment[0], segment[1], c0 / (c0 - c1));
        count += 1;

        if c1 < 0.0 Swap(*clipped[0], *clipped[1]); // Keep the original segment direction
    }

    return clipped, count;
}

clip_segment_with_polygon :: (segment: Segment, vertices: [] Vector2) -> Segment, int {
    clipped := segment;
    count: int;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        edge : Segment = .[ vertices[i], vertices[j] ];
        clipped, count = clip_segment_with_line(clipped, edge);

        if count != 2 break;
    }

    return clipped, count;
}

#scope_module

perpendicular :: (using v: Vector2) -> Vector2 #expand {
    return .{ -y, x };
}

cross :: (a: Vector2, b: Vector2) -> float #expand {
    return a.x * b.y - b.x * a.y;
}

Segment :: [2] Vector2;

segment_normal :: (segment: Segment) -> Vector2 {
    return perpendicular(normalize(segment[0] - segment[1]));
}