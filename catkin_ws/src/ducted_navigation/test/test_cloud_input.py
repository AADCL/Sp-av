import pathlib,sys,struct,unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'))
from ducted_navigation.cloud_input import read_xyz
from ducted_navigation.runtime import NavigationRuntime,RuntimeConfig,transform_points
import numpy as np

class CloudInputTest(unittest.TestCase):
    def cloud(self,bigendian=False):
        # Organized cloud with reordered fields, point padding and row padding.
        data=bytearray(104);order='>' if bigendian else '<'
        for row in range(2):
            for col in range(3):
                i=row*3+col
                struct.pack_into(order+'fff',data,row*52+col*16,float(i+2),float(i),float(i+1))
        return NS(width=3,height=2,point_step=16,row_step=52,is_bigendian=bigendian,data=data,
            fields=[NS(name=n,offset=o,datatype=7,count=1) for n,o in [('x',4),('y',8),('z',0)]])

    def test_padded_rows_and_field_offsets_in_both_byte_orders(self):
        for big in [False,True]:
            actual=read_xyz(self.cloud(big))
            np.testing.assert_array_equal(actual,[[i,i+1,i+2] for i in range(6)])

    def test_malformed_layout_never_becomes_obstacle_free_cloud(self):
        for alter in [lambda m:m.data.pop(),lambda m:setattr(m,'row_step',40),
                      lambda m:setattr(m.fields[0],'offset',14),
                      lambda m:setattr(m.fields[0],'datatype',2),
                      lambda m:m.fields.append(m.fields[0])]:
            msg=self.cloud();alter(msg)
            with self.assertRaises(ValueError):read_xyz(msg)

    def test_nonfinite_point_is_preserved_for_runtime_rejection(self):
        msg=self.cloud();struct.pack_into('<f',msg.data,4,float('nan'))
        points=read_xyz(msg)
        self.assertTrue(np.isnan(points[0,0]))
        runtime=NavigationRuntime(RuntimeConfig())
        self.assertFalse(runtime.accept_cloud(10,20,'odom',points,10))

    def test_runtime_owns_immutable_dense_array_without_tuple_roundtrip(self):
        points=np.arange(3000,dtype=float).reshape(-1,3)
        transformed=transform_points(points,(1,2,3),(0,0,0,1),as_array=True)
        runtime=NavigationRuntime(RuntimeConfig())
        self.assertTrue(runtime.accept_cloud(10,20,'odom',transformed,10))
        stored=runtime._records['cloud']['value']
        transformed[0,0]=999
        self.assertEqual(stored[0,0],1)
        with self.assertRaises(ValueError):stored[0,0]=888

if __name__=='__main__':unittest.main()
