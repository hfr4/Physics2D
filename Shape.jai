Shape :: struct {
    kind:       Shape_Kind;
    using data: Shape_Data;
}

Shape_Kind :: enum {
    CIRCLE;
    POLYGON;
}

Shape_Data :: union {
    circle:  Circle_Data;
    polygon: Polygon_Data;
}

Circle_Data :: struct {
    radius: float;
}

Polygon_Data :: struct {
    MAX_VERTICES :: 8;

    count:    int;
    vertices: [MAX_VERTICES] Vector2;
}

make_circle_shape :: (radius: float) -> Shape {
    shape: Shape;

    shape.kind = .CIRCLE;
    shape.circle.radius = radius;

    return shape;
}

make_polygon_shape :: (vertices: [] Vector2) -> Shape {
    shape: Shape;

    shape.kind = .POLYGON;
    shape.polygon.count = vertices.count;
    for vertices shape.polygon.vertices[it_index] = it;

    return shape;
}

get_vertices :: (using polygon: Polygon_Data) -> [] Vector2 {
    return .{ count, vertices.data };
}
