import sys
import os
import getopt
import subprocess
import glob
#from unv_io import stitchLayers2Unv

#===============================================================================
#                             deleteIfExists
#===============================================================================
def deleteIfExists(file):
    if os.path.exists(file):
        os.remove(file)
    else:
        print("The file does not exist")

#===============================================================================
#                                makeClean 
#===============================================================================
def makeClean():
    """
      Clean the working directory
    """
    stlFiles = glob.glob('*.stl')
    vtkFiles = glob.glob('*.vtk')
    offFiles = glob.glob('*.off')
    datFiles = glob.glob('*.dat')

    for item in stlFiles:
        deleteIfExists(item)

    for item in vtkFiles:
        deleteIfExists(item)

    for item in offFiles:
        deleteIfExists(item)

    for item in datFiles:
        deleteIfExists(item)

    deleteIfExists("outer.unv")
    deleteIfExists("myout.unv")

#===============================================================================
#                        usage function
#===============================================================================
def usage():
    """
    Print out usage information
    """
    print("Usage: python boundarylayer_generation.py [options]")
    print("Where options can be:")
    print("[-u', '--unv-mesh]: path to input .unv file")
    print("[-b', '--base-layer]: path to the .off file of the first boundary layer")
    print("[-r', '--orig-mesh]: path to the .dat file of the layer to which we want to attach the boundary layers")
    print("[-o', '--output-file]: name of the output unv file")
    print("[-s', '--salome-path]: path to the salome launcher")
    print("[-c', '--clean]: remote temporary and intermediate files from the working directory")
    print("[-t', '--thickness]: thickness of a boundary layer")
    print("[-l', '--layers]: the total number of boundary layers")
    print("[-h', '--help']: prints this message")
    print("Example: python3 ./unv_io.py -u netzsch_outer2.unv -b baseMeshLayer1.off -r RotorI3.dat")

def main():

    unvMesh = ""
    origMesh = ""
    baseLayer = ""
    numLayers = 2
    outputFile = "output.unv"
    salomePath = "/home/rafa/bin/SALOME-9.3.0-UB18.04-SRC/salome"

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'u:r:b:t:m:o:s:l:ch',
                                   ['unv-mesh=', 'orig-mesh=', 'base-layer=', 'thickness=', 'method=', 'output-file=',
                                    'salome-path=', 'layers=', 'clean',
                                    'help'])

    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-u', '--unv-mesh'):
            unvMesh = arg
        elif opt in ('-b', '--base-layer'):
            baseLayer = arg
        elif opt in ('-r', '--orig-mesh'):
            origMesh = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        elif opt in ('-m', '--method'):
            method = int(arg)
        elif opt in ('-o', '--output-file'):
            outputFile = arg
        elif opt in ('-s', '--salome-path'):
            salomePath = arg
        elif opt in ('-l', '--layers'):
            numLayers = int(opt)
        elif opt in ('-c', '--clean'):
            makeClean()
            sys.exit(2)
        else:
            usage()
            sys.exit(2)

    #stitchLayers2Unv(baseLayer, origMesh, unvMesh, outputFile)
    unvMeshAbsolute = os.getcwd() + '/' + unvMesh
    unvMeshOutAbsolute = os.getcwd() + '/start.unv' 
    workingDir = os.getcwd() 

    subprocess.call(['%s --shutdown-servers=1 -t salome_command_dump.py args:%s,%s,%s' % (salomePath, unvMeshAbsolute, unvMeshOutAbsolute, workingDir)], shell=True)
    subprocess.call(['python3 ./dat2off.py -i StatorI.dat -o statori.off'], shell=True)
    subprocess.call(['python3 ./gen_boundary_layers.py -s statori.off -t %f' %thickness], shell=True)
    subprocess.call(['python3 ./unv_io.py -u start.unv -b baseMeshLayer1.off -o StatorI.dat'], shell=True)

    # This step uses the Rotor group
    subprocess.call(['%s --shutdown-servers=1 -t salome_rotor.py args:%s' %(salomePath, workingDir)],shell=True)
    subprocess.call(['python3 ./dat2off.py -i RotorI.dat -o rotori.off'],shell=True)
    subprocess.call(['python3 ./gen_boundary_layers.py -s rotori.off -t %f' %thickness],shell=True)
    subprocess.call(['python3 ./unv_io.py -u outer.unv -b baseMeshLayer1.off -o RotorI.dat'],shell=True)
    subprocess.call(['%s --shutdown-servers=1 -t salome_final.py args:%s' %(salomePath, workingDir)],shell=True)

    # Here comes a last Salome step where the final groups are constructed

if __name__ == "__main__":
    main()
