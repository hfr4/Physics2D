ComputeCircleInertia :: (radius: float, mass: float) -> float {
    return mass * radius * radius / 2.0;
}

ComputePolygonInertia :: (vertices: [] Vector2, mass: float) -> float {
    inertia: float;

    centroid      := ComputePolygonCentroid(vertices);
    triangle_mass := mass / vertices.count;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        ci := vertices[i] - centroid;
        cj := vertices[j] - centroid;

        inertia += (dot(ci, ci) + dot(cj, cj) + dot(ci, cj)) * triangle_mass / 6.0;
    }

    return inertia;
}

ComputePolygonCentroid :: (vertices: [] Vector2) -> Vector2 {
    centroid: Vector2;
    area: float;

    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        partial_area := Cross(vertices[i], vertices[j]);

        area     += partial_area;
        centroid += partial_area * (vertices[i] + vertices[j]);
    }

    area     /= 2.0;
    centroid /= 6.0 * area;

    return centroid;
}

ClosestEdgeFromPoint :: (vertices: [] Vector2, point: Vector2) -> [2] Vector2, int {
    result: [2] Vector2;
	index: int;

    max_depth := -FLOAT32_MAX;
    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        edge   := Vector2.[ vertices[i], vertices[j] ];
		normal := Perpendicular(normalize(edge[0] - edge[1]));
        depth  := dot(point - edge[0], normal);

        if depth > max_depth {
            max_depth = depth;
            result    = edge;
			index     = i;
        }
    }

    return result, index;
}

IsPointInsidePolygon :: (vertices: [] Vector2, point: Vector2) -> bool {
    for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

        edge   := Vector2.[ vertices[i], vertices[j] ];
        normal := Perpendicular(normalize(edge[0] - edge[1]));
        depth  := dot(point - edge[0], normal);

        if depth > 0.0 return false;
    }

    return true;
}

ClosestPointOnEdge :: (edge: [2] Vector2, point: Vector2) -> Vector2 {
    vector := edge[1] - edge[0];
    ratio  := dot(point - edge[0], vector) / dot(vector, vector);
	Clamp(*ratio, 0.0, 1.0);
    return lerp(edge[0], edge[1], ratio);
}

ComputeReferentEdge :: (A: [] Vector2, B: [] Vector2) -> [2] Vector2, int, bool, bool {
	SAT :: (A: [] Vector2, B: [] Vector2) -> [2] Vector2, int, float {
		result: [2] Vector2;
		index: int;
		separation := -FLOAT32_MAX;

		for i: 0..A.count - 1 {
			j := (i + 1) % A.count;

			edge   := Vector2.[ A[i], A[j] ];
			normal := Perpendicular(normalize(edge[0] - edge[1]));

			min_proj := FLOAT32_MAX;
			for B  min_proj = min(min_proj, dot((it - edge[0]), normal));

			if separation < min_proj {
				separation = min_proj;
				result     = edge;
				index      = i;
			}
		}

		return result, index, separation;
	}

	A_edge, A_index, A_depth := SAT(A, B);
	if A_depth >= 0.0 return .[ .{}, .{} ], 0, false, false;

	B_edge, B_index, B_depth := SAT(B, A);
	if B_depth >= 0.0 return .[ .{}, .{} ], 0, false, false;

	if A_depth > B_depth return A_edge, A_index, true , true;
	else                 return B_edge, B_index, false, true;
}

ComputeIncidentEdge :: (vertices: [] Vector2, referent_edge_normal: Vector2) -> [2] Vector2, int {
	result: [2] Vector2;
	index: int;
	min_proj := FLOAT32_MAX;

	for i: 0..vertices.count - 1 {
        j := (i + 1) % vertices.count;

		edge   := Vector2.[ vertices[i], vertices[j] ];
		normal := Perpendicular(normalize(edge[0] - edge[1]));
		proj   := dot(referent_edge_normal, normal);

		if proj < min_proj {
			min_proj = proj;
			result   = edge;
			index    = i;
		}
	}

	return result, index;
}

ClipEdgeWithLine :: (edge: *[2] Vector2, position: Vector2, direction: Vector2) -> int {
    clipped: [2] Vector2;
    count: int;

    c0 := Cross(edge.*[0] - position, direction);
    c1 := Cross(edge.*[1] - position, direction);

    if c0 <= 0.0 { clipped[count] = edge.*[0]; count += 1; }
    if c1 <= 0.0 { clipped[count] = edge.*[1]; count += 1; }

    if c0 * c1 < 0.0 {
		ratio := c0 / (c0 - c1);
        clipped[count] = lerp(edge.*[0], edge.*[1], ratio);
        count += 1;
    }

	edge.* = clipped;

    return count;
}

Inverse :: (using m: Matrix2) -> Matrix2 {
	det := _11 * _22 - _21 * _12;
	assert(det != 0.0);
	det = 1.0 / det;
	return det * Matrix2.{ _22, -_12, -_21, _11 };
}

Perpendicular :: (using v: Vector2) -> Vector2 {
	return .{ -y, x };
}

Cross :: (a: Vector2, b: Vector2) -> float {
	return a.x * b.y - a.y * b.x;
}