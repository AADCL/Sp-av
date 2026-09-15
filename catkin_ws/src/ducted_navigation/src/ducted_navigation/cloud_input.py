"""Decode XYZ without allocating a Python tuple for every point and axis."""
import numpy as np


def read_xyz(message):
    width,height=int(message.width),int(message.height)
    step,row=int(message.point_step),int(message.row_step)
    if (width<0 or height<0 or width*height>200000 or step<12 or step>128
            or row<width*step or len(message.data)!=row*height):
        raise ValueError('invalid or oversized cloud layout')
    result=np.empty((width*height,3),dtype=np.float64)
    for axis,name in enumerate(('x','y','z')):
        fields=[f for f in message.fields if f.name==name]
        if len(fields)!=1:
            raise ValueError('missing or duplicate XYZ field')
        field=fields[0]
        size={7:4,8:8}.get(field.datatype)
        if size is None or field.count!=1 or field.offset<0 or field.offset+size>step:
            raise ValueError('invalid XYZ field layout')
        if not width*height:continue
        dtype=('>' if message.is_bigendian else '<')+('f4' if size==4 else 'f8')
        values=np.ndarray((height,width),dtype=dtype,buffer=message.data,
                          offset=field.offset,strides=(row,step))
        result[:,axis]=values.reshape(-1)
    # Do not drop NaNs: downstream validation must reject malformed input.
    return result
