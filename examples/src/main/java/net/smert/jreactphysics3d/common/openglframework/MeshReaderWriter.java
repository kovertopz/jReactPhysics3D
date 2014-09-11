package net.smert.jreactphysics3d.common.openglframework;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector2;
import net.smert.jreactphysics3d.common.openglframework.maths.Vector3;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class MeshReaderWriter {

    private static final List<Boolean> faceIsQuad = new ArrayList<>();
    private static final List<Integer> normalIndices = new ArrayList<>();
    private static final List<Integer> uvIndices = new ArrayList<>();
    private static final List<Integer> vertexIndices = new ArrayList<>();
    private static final List<Vector3> normals = new ArrayList<>();
    private static final List<Vector2> uvs = new ArrayList<>();
    private static final List<Vector3> vertices = new ArrayList<>();

    // Constructor (private because we do not want instances of this class)
    private MeshReaderWriter() {
    }

    protected static void parse(String line) {
        String[] words = line.split(" ");
        int numberOfWords = words.length;

        if (numberOfWords == 0) {
            return;
        }

        String[] parameters = new String[numberOfWords - 1];
        for (int i = 0, n = 1; n < numberOfWords; i++, n++) {
            parameters[i] = words[n];
        }
        int numberOfParameters = parameters.length;

        float v1, v2, v3;

        String firstWord = words[0];
        switch (firstWord) {

            case "f":
                if ((numberOfParameters < 3) && (numberOfParameters > 4)) {
                    return;
                }

                boolean isQuad = false;

                if (numberOfParameters == 4) {
                    isQuad = true;
                }

                faceIsQuad.add(isQuad);

                int slashCount = parameters[0].length() - parameters[0].replace("/", "").length();

                for (int i = 0; i < numberOfParameters; i++) {
                    int firstSlash, i1, i2, i3, secondSlash;
                    String normalIndex, uvIndex, vertexIndex;

                    String parameter = parameters[i];

                    switch (slashCount) {
                        // Vertex Index
                        case 0:
                            i1 = Integer.parseInt(parameter) - 1;

                            vertexIndices.add(i1);
                            break;

                        // Vertex Index / UV Index
                        case 1:
                            firstSlash = parameter.indexOf("/");

                            vertexIndex = parameter.substring(0, firstSlash);
                            uvIndex = parameter.substring(firstSlash + 1);

                            i1 = Integer.parseInt(vertexIndex) - 1;
                            i2 = Integer.parseInt(uvIndex) - 1;

                            vertexIndices.add(i1);
                            uvIndices.add(i2);
                            break;

                        // Vertex Index / UV Index / Normal Index 
                        case 2:
                            firstSlash = parameter.indexOf("/");
                            secondSlash = parameter.indexOf("/", firstSlash + 1);

                            vertexIndex = parameter.substring(0, firstSlash);
                            uvIndex = parameter.substring(firstSlash + 1, secondSlash);
                            normalIndex = parameter.substring(secondSlash + 1);

                            i1 = Integer.parseInt(vertexIndex) - 1;
                            i2 = Integer.parseInt(uvIndex) - 1;
                            i3 = Integer.parseInt(normalIndex) - 1;

                            vertexIndices.add(i1);
                            uvIndices.add(i2);
                            normalIndices.add(i3);
                            break;

                        default:
                            throw new IndexOutOfBoundsException("Invalid number of slashes found for face: " + line);
                    }
                }

                break;

            case "v":
                if (numberOfParameters != 3) {
                    return;
                }

                v1 = Float.parseFloat(parameters[0]);
                v2 = Float.parseFloat(parameters[1]);
                v3 = Float.parseFloat(parameters[2]);

                vertices.add(new Vector3(v1, v2, v3));
                break;

            case "vn":
                if (numberOfParameters != 3) {
                    return;
                }

                v1 = Float.parseFloat(parameters[0]);
                v2 = Float.parseFloat(parameters[1]);
                v3 = Float.parseFloat(parameters[2]);

                normals.add(new Vector3(v1, v2, v3));
                break;

            case "vt":
                if (numberOfParameters != 3) {
                    return;
                }

                v1 = Float.parseFloat(parameters[0]);
                v2 = Float.parseFloat(parameters[1]);

                uvs.add(new Vector2(v1, v2));
                break;
        }
    }

    protected static void readMeshFromJAR(String file) {
        // Open the file
        try {
            InputStream inputstream = MeshReaderWriter.class.getResourceAsStream(file);
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputstream));
            String line;

            // ---------- Collect the data from the file ---------- //
            // For each line of the file
            while ((line = reader.readLine()) != null) {
                MeshReaderWriter.parse(line);
            }

            reader.close();
            inputstream.close();
        } catch (IOException e) {   // If we cannot open the file
            // Throw an exception and display an error message
            throw new RuntimeException("Unable to open mesh file: " + file);
        }
    }

    // Load an OBJ file with a triangular or quad mesh
    private static void loadOBJFile(String filename, Mesh meshToCreate) {
        faceIsQuad.clear();
        normalIndices.clear();
        uvIndices.clear();
        vertexIndices.clear();
        normals.clear();
        uvs.clear();
        vertices.clear();

        readMeshFromJAR(filename);

        assert (!vertexIndices.isEmpty());
        assert (normalIndices.isEmpty() || normalIndices.size() == vertexIndices.size());
        assert (uvIndices.isEmpty() || uvIndices.size() == vertexIndices.size());

        // ---------- Merge the data that we have collected from the file ---------- //
        // Destroy the current mesh
        meshToCreate.destroy();

        // Mesh data
        List<Integer> meshIndices = new ArrayList<>();
        List<Vector3> meshNormals = new ArrayList<>();
        List<Vector2> meshUVs = new ArrayList<>();
        List<Vector3> meshVertices = new ArrayList<>();

        // We cannot load mesh with several parts for the moment
        int meshPart = 0;

        // Fill in the vertex indices
        // We also triangulate each quad face
        for (int i = 0, j = 0; i < vertexIndices.size(); j++) {

            // Get the current vertex IDs
            int i1 = vertexIndices.get(i);
            int i2 = vertexIndices.get(i + 1);
            int i3 = vertexIndices.get(i + 2);

            // Add the vertex normal
            if (!normalIndices.isEmpty() && !normals.isEmpty()) {
                meshNormals.add(i1, normals.get(normalIndices.get(i)));
                meshNormals.add(i2, normals.get(normalIndices.get(i + 1)));
                meshNormals.add(i3, normals.get(normalIndices.get(i + 2)));
            }

            // Add the vertex UV texture coordinates
            if (!uvIndices.isEmpty() && !uvs.isEmpty()) {
                meshUVs.add(i1, uvs.get(uvIndices.get(i)));
                meshUVs.add(i2, uvs.get(uvIndices.get(i + 1)));
                meshUVs.add(i3, uvs.get(uvIndices.get(i + 2)));
            }

            // If the current vertex not in a quad (it is part of a triangle)
            if (!faceIsQuad.get(j)) {

                // Add the vertex indices
                meshIndices.add(i1);
                meshIndices.add(i2);
                meshIndices.add(i3);

                i += 3;
            } else {  // If the current vertex is in a quad

                Vector3 v1 = vertices.get(i1);
                Vector3 v2 = vertices.get(i2);
                Vector3 v3 = vertices.get(i3);
                int i4 = vertexIndices.get(i + 3);
                Vector3 v4 = vertices.get(i4);

                Vector3 v13 = v3.operatorSubtract(v1);
                Vector3 v12 = v2.operatorSubtract(v1);
                Vector3 v14 = v4.operatorSubtract(v1);

                float a1 = v13.dot(v12);
                float a2 = v13.dot(v14);
                if ((a1 >= 0 && a2 <= 0) || (a1 <= 0 && a2 >= 0)) {
                    meshIndices.add(i1);
                    meshIndices.add(i2);
                    meshIndices.add(i3);
                    meshIndices.add(i1);
                    meshIndices.add(i3);
                    meshIndices.add(i4);
                } else {
                    meshIndices.add(i1);
                    meshIndices.add(i2);
                    meshIndices.add(i4);
                    meshIndices.add(i2);
                    meshIndices.add(i3);
                    meshIndices.add(i4);
                }

                // Add the vertex normal
                if (!normalIndices.isEmpty() && !normals.isEmpty()) {
                    meshNormals.add(i4, normals.get(normalIndices.get(i)));
                }

                // Add the vertex UV texture coordinates
                if (!uvIndices.isEmpty() && !uvs.isEmpty()) {
                    meshUVs.add(i4, uvs.get(uvIndices.get(i)));
                }

                i += 4;
            }
        }

        assert (meshNormals.isEmpty() || meshNormals.size() == vertices.size());
        assert (meshUVs.isEmpty() || meshUVs.size() == vertices.size());

        meshVertices.addAll(vertices);

        // Set the data to the mesh
        meshToCreate.setIndices(meshIndices);
        meshToCreate.setVertices(meshVertices);
        meshToCreate.setNormals(meshNormals);
        meshToCreate.setUVs(meshUVs);
    }

    // Store a mesh into a OBJ file
    private static void writeOBJFile(String filename, Mesh meshToWrite) {
    }

    // Load a mesh from a file and returns true if the mesh has been sucessfully loaded
    public static void loadMeshFromFile(String filename, Mesh meshToCreate) {

        // Get the extension of the file
        int startPosExtension = filename.lastIndexOf(".");
        String extension = filename.substring(startPosExtension + 1);

        // Load the file using the correct method
        if (extension.equals("obj")) {
            loadOBJFile(filename, meshToCreate);
        } else {

            // Display an error message and throw an exception
            throw new IllegalArgumentException("Error : the MeshReaderWriter class cannot load a file with the extension ." + extension);
        }
    }

    // Write a mesh to a file
    public static void writeMeshToFile(String filename, Mesh meshToWrite) {

        // Get the extension of the file
        int startPosExtension = filename.lastIndexOf(".");
        String extension = filename.substring(startPosExtension + 1);

        // Load the file using the correct method
        if (extension.equals("obj")) {
            writeOBJFile(filename, meshToWrite);
        } else {

            // Display an error message and throw an exception
            throw new IllegalArgumentException("Error : the MeshReaderWriter class cannot store a mesh file with the extension ." + extension);
        }
    }

}
