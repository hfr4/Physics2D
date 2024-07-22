Shape_Kind :: enum {
    CIRCLE;
    POLYGON;
}

Polygon_Shape :: struct {
    MAX_VERTICES_PER_POLYGON :: 8;

    vertices: [MAX_VERTICES_PER_POLYGON] Vector2;
    count:    int;
}

make_polygon_shape :: (vertices: [] Vector2) -> Polygon_Shape {
    polygon: Polygon_Shape;

    for vertices {
        polygon.vertices[it_index] = it;
    }
    polygon.count = vertices.count;

    return polygon;
}

get_vertices :: (using polygon: Polygon_Shape) -> [] Vector2 {
    return .{ count, vertices.data };
}

Circle_Shape :: struct {
    radius: float;
}

make_circle_shape :: (radius: float) -> Circle_Shape {
    circle: Circle_Shape;

    circle.radius = radius;

    return circle;
}

