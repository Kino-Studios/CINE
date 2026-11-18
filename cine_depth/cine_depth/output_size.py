"""Sizes the published depth image, and the camera intrinsics that go with that size."""


def output_size(
    input_width: int, input_height: int, requested_width: int, requested_height: int
) -> tuple[int, int]:
    """Returns (width, height); 0 for a requested value means it is not requested.

    With neither requested the input size is kept; with one, the input aspect ratio is.
    """
    if requested_width and requested_height:
        return requested_width, requested_height
    if requested_width:
        return requested_width, round(input_height * requested_width / input_width)
    if requested_height:
        return round(input_width * requested_height / input_height), requested_height

    return input_width, input_height


def scaled_intrinsics(
    focal_x: float,
    focal_y: float,
    centre_x: float,
    centre_y: float,
    input_width: int,
    output_width: int,
) -> tuple[float, float, float, float]:
    scale = output_width / input_width

    return focal_x * scale, focal_y * scale, centre_x * scale, centre_y * scale
