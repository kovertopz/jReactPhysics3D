package net.smert.jreactphysics3d.common.openglframework;

/**
 * This class is used to load or write a texture image in different picture format. It currently allows to read and
 * write the following formats : .tga
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class TextureReaderWriter {

    // JPEG quality for compression (in percent)
    static int JPEG_COMPRESSION_QUALITY = 98;

    // Constructor (private because we do not want instances of this class)
    private TextureReaderWriter() {
    }

    // Read a TGA picture
    private static void readTGAPicture(String filename, Texture2D textureToCreate) {
    }

    // Write a TGA picture
    private static void writeTGAPicture(String filename, Texture2D texture) {
    }

    // Read a JPEG picture
    private static void readJPEGPicture(String filename, Texture2D textureToCreate) {
    }

    // Write a JPEG picture
    private static void writeJPEGPicture(String filename, Texture2D texture) {
    }

    // Load a texture from a file
    public void loadTextureFromFile(String filename, Texture2D textureToCreate) {

        // Get the extension of the file
        int startPosExtension = filename.lastIndexOf(".");
        String extension = filename.substring(startPosExtension + 1);

        // Load the file using the correct method
        switch (extension) {
            case "tga":
                readTGAPicture(filename, textureToCreate);
                break;
            case "jpg":
            case "jpeg":
                readJPEGPicture(filename, textureToCreate);
                break;
            default:
                // Display an error message and throw an exception
                throw new IllegalArgumentException("Error : the TextureLoader class cannot load a file with the extension ." + extension);
        }
    }

    // Write a texture to a file
    public void writeTextureToFile(String filename, Texture2D texture) {

        // Get the extension of the file
        int startPosExtension = filename.lastIndexOf(".");
        String extension = filename.substring(startPosExtension + 1);

        // Write the file using the correct method
        switch (extension) {
            case "tga":
                writeTGAPicture(filename, texture);
                break;
            case "jpg":
            case "jpeg":
                writeJPEGPicture(filename, texture);
                break;
            default:
                // Display an error message and throw an exception
                throw new IllegalArgumentException("Error : the TextureReaderWriter class cannot write a file with the extension ." + extension);
        }
    }

}
