from zmqRemoteApi import RemoteAPIClient

def printHTM(mat):
    for i in range(0, 9, 4):
        print ("| %.5f \t%.5f \t%.5f \t%.5f \t|" % (mat[i], mat[i+1], mat[i+2], mat[i+3]))
    print ("| %.5f \t%.5f \t%.5f \t%.5f \t|" % (0, 0, 0, 1))
   
client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)
sim.startSimulation()

dummy = sim.getObject('/Dummy')
dummy_position = sim.getObjectPosition(dummy, -1)
dummy_orientation = sim.getObjectOrientation(dummy, -1)
dummy_trans_matrix = sim.getObjectMatrix(dummy, -1)
print("\nDummy Position: ", dummy_position)
print("\nDummy Orientation: ", dummy_orientation)
print("\nDummy Transformation Matrix: ")
printHTM(dummy_trans_matrix)

sim.stopSimulation()