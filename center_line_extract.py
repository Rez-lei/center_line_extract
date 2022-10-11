'''
Author: xdw zl
Date: 2022-10-11 09:39:51
LastEditors: xdw dongdevelop@outlook.com
LastEditTime: 2022-10-11 11:55:16
Version: 0.0.2
Description: 道路中心线提取Python程序代码
'''

import sys
from osgeo import gdal, ogr, osr
import numpy as np
import cv2
from skimage import morphology
import json

'''
description: 主函数 参数1: 输入文件位置 参数2: 输出文件位置
return {*}
'''
def main():
  in_path = sys.argv[1]
  out_path = sys.argv[2]
  # get_geo_transform(ref_path)
  f = open(in_path, "r", encoding="utf-8")
  m = json.load(f)
  r = json.dumps(m, ensure_ascii=False)
  img_data, geo_transform = vector_to_raster(r)
  road_data = extract_road(img_data)
  raster_to_vector(road_data, out_path, geo_transform)

def get_geo_transform(ref_path):
  in_img = gdal.Open(ref_path)
  geo_transform = in_img.GetGeoTransform()
  print("geo_transform:", geo_transform)

'''
description: 输入矢量数据转换为栅格数据
param {*} vector 输入矢量二进制数据流
return {*}
'''
def vector_to_raster(vector):
  geojson_driver = ogr.GetDriverByName("GeoJSON")
  # 将输入文件数据写入gdal虚拟内存文件系统
  gdal.FileFromMemBuffer("/vsimem/inVector", bytes(vector, encoding="utf-8"))
  geojson_ds = geojson_driver.Open("/vsimem/inVector", 1)
  v_layer = geojson_ds.GetLayer()
  lon_min, lon_max, lat_min, lat_max = v_layer.GetExtent()
  top_left = [lon_min, lat_max]
  bottom_right = [lon_max, lat_min]
  s_proj = str(v_layer.GetSpatialRef())

  dst_transform = (lon_min, 1, 0, lat_max, 0, -1)
  d_lon = int(abs((lon_max - lon_min) / dst_transform[1]))
  d_lat = int(abs((lat_max - lat_min) / dst_transform[5]))
  target_ds = gdal.GetDriverByName("GTiff").Create("/vsimem/outImage", d_lon, d_lat, 1, gdal.GDT_Byte,
                                                     options=['COMPRESS=LZW', 'BIGTIFF=YES'])
  target_ds.SetGeoTransform(dst_transform)
  target_ds.SetProjection(s_proj)
  band = target_ds.GetRasterBand(1)
  NoData_value = 0
  band.SetNoDataValue(NoData_value)
  band.FlushCache()

  if len(v_layer) > 0:
    gdal.RasterizeLayer(target_ds, [1], v_layer, burn_values=[1])
  y_buffer = band.ReadAsArray()
  target_ds.WriteRaster(0, 0, d_lon, d_lat, y_buffer.tobytes())
  target_ds = None
  del target_ds, v_layer
  # 从gdal虚拟内存文件中读取数据流
  bin_data = gdal.VSIGetMemFileBuffer_unsafe("/vsimem/outImage")
  img_buffer = np.frombuffer(bin_data, np.uint8)
  return img_buffer, dst_transform

'''
description: 提取道路中心线，返回结果图像文件数据
param {*} tif_data 输入栅格文件数据
return {*}
'''
def extract_road(tif_data):
  img = cv2.imdecode(tif_data, 0)
  img[img == 255] = 1
  skeleton0 = morphology.skeletonize(img)
  skeleton = skeleton0.astype(np.uint8) * 255
  result = cv2.imencode(".tif",skeleton)
  return result[1]

'''
description: 获取连通域
param {*} img_data opencv读取的图片数据数组
return {*} 连通域(拐点坐标)
'''
def get_contour(img_data):
  img_gray = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
  ret, img_bin = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
  contours, hierarchy = cv2.findContours(
    img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  return img_gray, contours

'''
description: 将栅格数据转为矢量文件
param {*} line_tif 输入栅格数据
param {*} out_path 输出矢量文件绝对路径
return {*}
'''
def raster_to_vector(line_tif, out_path, geo_transform):
  img_src = cv2.imdecode(line_tif, 1)
  img_gray, contours = get_contour(img_src)

  gdal.SetConfigOption("GDAL_FILENAME_IS_UTF8", "NO")
  gdal.SetConfigOption("SHAPE_ENCODING", "CP936")
  ogr.RegisterAll()
  out_driver = ogr.GetDriverByName("GeoJSON")
  out_ds = out_driver.CreateDataSource(out_path)
  srs = osr.SpatialReference()
  srs.ImportFromEPSG(4529)

  out_layer = out_ds.CreateLayer("line", srs, geom_type=ogr.wkbLineString)
  out_field_id = ogr.FieldDefn("FieldID", ogr.OFTInteger)
  out_layer.CreateField(out_field_id, 1)
  out_field_name = ogr.FieldDefn("FieldName", ogr.OFTString)
  out_field_name.SetWidth(100)
  out_layer.CreateField(out_field_name, 1)

  out_feature_defn = out_layer.GetLayerDefn()

  # 创建一条线
  out_feature = ogr.Feature(out_feature_defn)
  out_feature.SetField(0, 0)
  out_feature.SetField(1, "line1")

  for contour in contours:
    box1 = ogr.Geometry(ogr.wkbLinearRing)
    for point in contour:
      x_col = geo_transform[0] + geo_transform[1]*(float(point[0, 0])) + (float(point[0, 1])) * geo_transform[2]
      y_row = geo_transform[3] + geo_transform[4] * (float(point[0, 0])) + (float(point[0, 1])) * geo_transform[5]
      box1.AddPoint(x_col, y_row)
    out_feature.SetGeometry(box1)
    out_layer.CreateFeature(out_feature)

  ring = ogr.Geometry(ogr.wkbLinearRing)
  for i in range(10):
    wkt = "LINESTRING(%f %f,%f %f)" % (
      float(0), float(1000 + i * 10), float(1000), float(1000 + i * 10))
    point = ogr.CreateGeometryFromWkt(wkt)
    out_feature.SetGeometry(point)
    out_feature.SetGeometry(ring)
    out_layer.CreateFeature(out_feature)

  out_feature = None
  out_ds = None

if __name__ == "__main__":
  main()
