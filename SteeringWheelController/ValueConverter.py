
def linear_converter(min, max, in_value, invert=False):
    """
    :param min: Minimal ouput value
    :param max: Max output value
    :param in_value: input value in the interval [-1,1]
    :return: ozutput value
    """
    if invert:
        in_value = -1 * in_value
    return (max + min) / 2 + in_value * (max - min) / 2
