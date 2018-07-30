import gdal, ogr, os, osr
import numpy
import subprocess



def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print ('Error: Creating directory. ' +  directory)
        


def polygon (imagepath):
    bag = gdal.Open(imagepath)  # replace it with your file
     # raster is projected                                          
    bag_gtrn = bag.GetGeoTransform()
    bag_proj = bag.GetProjectionRef()
    bag_srs = osr.SpatialReference(bag_proj)
    geo_srs =bag_srs.CloneGeogCS()                 # new srs obj to go from x,y -> φ,λ
    transform = osr.CoordinateTransformation( bag_srs, geo_srs)

    bag_bbox_cells = (
            (0., 0.),
            (0, bag.RasterYSize),
            (bag.RasterXSize, bag.RasterYSize),
            (bag.RasterXSize, 0),
            )

    geo_pts = []
    for x, y in bag_bbox_cells:
        x2 = bag_gtrn[0] + bag_gtrn[1] * x + bag_gtrn[2] * y
        y2 = bag_gtrn[3] + bag_gtrn[4] * x + bag_gtrn[5] * y
        geo_pts.append(x2)
        geo_pts.append(y2)
    #geo_pt = transform.TransformPoint(x2, y2)[:2]
    #geo_pts.append(geo_pt)
        

    mspoly = ogr.Geometry(ogr.wkbLinearRing)
    mspoly.AddPoint(geo_pts[0], geo_pts[1])
    mspoly.AddPoint(geo_pts[2], geo_pts[3])
    mspoly.AddPoint(geo_pts[4], geo_pts[5])
    mspoly.AddPoint(geo_pts[6], geo_pts[7])
    mspoly.AddPoint(geo_pts[0], geo_pts[1])

    poli1 = ogr.Geometry(ogr.wkbPolygon)
    poli1.AddGeometry(mspoly)
    return poli1

def intersect(poli_ms,poli_pan):
    wkt1 = str(poli_ms)
    wkt2 = str(poli_pan)

    poly1 = ogr.CreateGeometryFromWkt(wkt1)
    poly2 = ogr.CreateGeometryFromWkt(wkt2)
    intersection = poly1.Intersection(poly2)
    ring = intersection.GetGeometryRef(0)
    count = ring.GetPointCount()
    countlist=[]
    for i in range(0,count):
        countlist+=[list(ring.GetPoint(i))]
    #for i in range(0,count):
        #print("x:"+str(ring.GetPoint(i)[0])+" y:"+str(ring.GetPoint(i)[1]))
        
    xdif=countlist[0][0]-countlist[3][0]
    ydif=countlist[0][1]-countlist[1][1]
    return xdif,ydif,countlist





def tif_to_tif (imagepath,output):
     
         os.system('gdal_translate -ot UInt16 -of GTiff '+imagepath+' '+output+'trans.tif')

def array2raster(newRasterfn,oX,oY,pixelWidth,pixelHeight,array,proj,nob):

    cols = array.shape[1]
    rows = array.shape[0]
    originX = oX
    originY = oY

    driver = gdal.GetDriverByName('GTiff')
    outRaster = driver.Create(newRasterfn, cols, rows, nob, gdal.GDT_UInt16)
    outRaster.SetGeoTransform((originX, pixelWidth, 0, originY, 0, pixelHeight))
    outband = outRaster.GetRasterBand(1)
    outband.WriteArray(array)
    outRasterSRS = osr.SpatialReference()
    outRasterSRS.ImportFromEPSG(4326)
    outRaster.SetProjection(proj)
    outband.FlushCache()

    
def origin(dif,geotrans,tilesize,i,j):
    x = geotrans[0] + geotrans[1]*i
    y = geotrans[3] + geotrans[5]*j
    return x,y
def divide(output,nob,outpath):
    for i in range (1,nob+1):
        os.system('gdal_translate -ot UInt16 -b '+str(i)+' -of GTiff  '+output+' '+outpath+str(i)+'.tif')


def func(imagepath,outpath,tilesize):

    #tif_to_tif(imagepath,outpath)
    image = gdal.Open(imagepath)
    #band = image.GetRasterBand(1)
    #xsize = band.XSize
    #ysize = band.YSize
    proj = image.GetProjection()
    geotrans = image.GetGeoTransform()
    pixel_x=int((dif[2][1][0]-geotrans[0])/geotrans[1])
    pixel_y=(int((dif[2][1][1]-geotrans[3])/geotrans[5]))
    nob = image.RasterCount
    for i in range (1,4):
        createFolder(outpath+"\\"+str(i))
    divide(imagepath,nob,outpath+"\\1\\")
    print(geotrans)
    print(pixel_y, pixel_x)
   
        
    
    
    
    
    
    #xrem = xsize % tilesize
    #yrem = ysize % tilesize
    #x = xsize - xrem
    #y = ysize - yrem
    for k in range (1,nob+1):
        divimage = gdal.Open(outpath+"\\1\\"+str(k)+'.tif')
        band = divimage.GetRasterBand(1)
        xsize = abs(dif[0]/geotrans[1])
        ysize = abs(dif[1]/geotrans[1])
        xrem = xsize % tilesize
        yrem = ysize % tilesize
        x =int( xsize - xrem)
        y =int( ysize - yrem)
        u=0
        v=0
        for i  in range (pixel_x, x, tilesize):
            u=u+1
            for j in range (pixel_y, y, tilesize):
                dataraster = band.ReadAsArray(i, j, tilesize, tilesize)
                oX,oY = origin(dif,geotrans,tilesize,i,j)
                array2raster(outpath+"\\2\\newRaster"+str(k)+"-"+str(u)+"-"+str(v)+".tif",oX,oY,geotrans[1],geotrans[5],dataraster,proj,1)
                v=v+1
    k = 1
    if nob>1:
        u=0
        v=0
        c=0
        b=0
        for i in range (pixel_x, x, tilesize):
            u=u+1
            c=c+1
            for j in range (pixel_y, y, tilesize):
                gm = os.path.join('C:\\','Users','USER-1','Desktop','gdal_merge.py')
                
                merge_command = ["python", gm, "-o", outpath+"\\3\\"+str(1)+"-"+str(c)+"-"+str(b)+".tif" ,"-co","GTiff","-separate"]
                b=b+1
                
                for o in range(1,nob+1):
                    merge_command+=[outpath+"\\2\\newRaster"+str(o)+"-"+str(u)+"-"+str(v)+".tif"]
                
                v=v+1
                #merge_command = ["python", gm, "-o", outpath+"\\3\\newRaster"+"merged"+"-"+str(i)+"-"+str(j)+".tif" ,"-co","GTiff","-separate", outpath+"\\2\\newRaster"+str(k+2)+"-"+str(i)+"-"+str(j)+".tif", outpath+"\\2\\newRaster"+str(k+1)+"-"+str(i)+"-"+str(j)+".tif", outpath+"\\2\\newRaster"+str(k)+"-"+str(i)+"-"+str(j)+".tif", outpath+"\\2\\newRaster"+str(k+3)+"-"+str(i)+"-"+str(j)+".tif"]
                
                
                print(merge_command)
                
                subprocess.call(merge_command,shell=True)


path_pan = "C:\\Users\\USER-1\\Desktop\\g\\PHR1A_ISTANBUL_20151224_PAN_ORT\\PHR1A_ISTANBUL_20151224_PAN_ORT\\IMG_PHR1A_P_001\\IMG_PHR1A_P_201512240903304_ORT_PHR1A_20180702_12090717g0otitmh09b_1_R1C1.JP2"      
path_ms = "C:\\Users\\USER-1\\Desktop\\g\\PHR1A_ISTANBUL_20151224_XS_ORT\\PHR1A_ISTANBUL_20151224_XS_ORT\\IMG_PHR1A_MS_001\\IMG_PHR1A_MS_201512240903304_ORT_PHR1A_20180702_1209161ogdvi94vm2rg_1_R1C1.JP2"
outpath_pan ="C:\\Users\\USER-1\\Desktop\\Uhuzam\\LandsatGörüntü\\pan"
outpath_ms="C:\\Users\\USER-1\\Desktop\\Uhuzam\\LandsatGörüntü\\ms"
poli1=polygon(path_pan)
poli2=polygon(path_ms)
dif=intersect(poli2,poli1)
print(dif)

func (path_pan,outpath_pan,256)
func (path_ms,outpath_ms,64)
