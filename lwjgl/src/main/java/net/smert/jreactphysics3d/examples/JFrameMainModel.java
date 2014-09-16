package net.smert.jreactphysics3d.examples;

import com.jdotsoft.jarloader.JarClassLoader;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.CodeSource;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import javax.swing.AbstractListModel;
import javax.swing.DefaultListModel;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class JFrameMainModel {

    private final JarClassLoader jarClassLoader;
    private final String exampleDirectory;
    private final String rootDirectoryPrefix;

    public JFrameMainModel(String exampleDirectory, String rootDirectoryPrefix) {
        jarClassLoader = new JarClassLoader();
        this.exampleDirectory = exampleDirectory;
        this.rootDirectoryPrefix = rootDirectoryPrefix;
    }

    private boolean checkMainMethodExists(String clazzName) {
        boolean result = false;

        try {
            Class clazz = jarClassLoader.loadClass(clazzName);
            Method method = clazz.getMethod("main", new Class[]{String[].class});

            boolean validModifiers = false;
            boolean validVoid = false;

            if (method != null) {
                method.setAccessible(true);
                int nModifiers = method.getModifiers();
                validModifiers = Modifier.isPublic(nModifiers) && Modifier.isStatic(nModifiers);
                Class<?> clazzRet = method.getReturnType();
                validVoid = (clazzRet == void.class);
            }
            if (validModifiers && validVoid) {
                result = true;
            }
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
            System.exit(-1);
        } catch (NoSuchMethodException | SecurityException e) {
            // Do nothing since not all classes have main methods.
        }

        return result;
    }

    private void filterFileAndCheck(String filename, String fullPath, String separator, List<String> mainClasses) {
        if (!filename.contains("$") && filename.endsWith(".class") && !filename.startsWith("Main")) {
            String clazzName = getClassNameFromPath(fullPath, separator);

            if ((clazzName.length() != 0) && (checkMainMethodExists(clazzName) == true)) {
                mainClasses.add(clazzName);
            }
        }
    }

    private String getClassNameFromPath(String fullPath, String separator) {
        String clazzName = "";
        String directory = (rootDirectoryPrefix + exampleDirectory).replace("/", separator);
        int index = fullPath.indexOf(directory);

        if (index != -1) {
            clazzName = fullPath.substring(index).replaceFirst(".class$", "").replace(separator, ".");
        }

        return clazzName;
    }

    private int getDirectoryDepth(String fullPathThisClass) {
        int depth = 0;
        fullPathThisClass = fullPathThisClass.replace("\\", "/");
        int index = fullPathThisClass.indexOf(rootDirectoryPrefix);

        if (index != -1) {
            String rootPathToClass = fullPathThisClass.substring(index);
            depth = rootPathToClass.length() - rootPathToClass.replace("/", "").length();
        }

        return depth;
    }

    private List<String> getMainClasses() {
        List<String> mainClasses = new ArrayList();
        URL classLocation = JFrameMainModel.class.getResource(JFrameMainModel.class.getSimpleName() + ".class");
        String protocol = classLocation.getProtocol();

        switch (protocol) {
            case "file":
                getMainClassesFromFile(mainClasses, classLocation);
                break;

            case "jar":
                getMainClassesFromJar(mainClasses);
                break;

            default:
                throw new RuntimeException("Unknown protocol for current class: " + protocol);
        }

        return mainClasses;
    }

    private void getMainClassesFromFile(List<String> mainClasses, URL classLocation) {
        try {
            Path path = Paths.get(classLocation.toURI());
            String rootPath = getRootPath(path.toString());
            listFilesInDirectory(rootPath, mainClasses);
        } catch (URISyntaxException e) {
            e.printStackTrace();
            System.exit(-1);
        }
    }

    private void getMainClassesFromJar(List<String> mainClasses) {
        CodeSource src = JFrameMainModel.class.getProtectionDomain().getCodeSource();

        if (src != null) {
            URL jar = src.getLocation();
            ZipInputStream zip;

            try {
                zip = new ZipInputStream(jar.openStream());

                while (true) {
                    ZipEntry e = zip.getNextEntry();

                    if (e == null) {
                        break;
                    }

                    String fullPath = e.getName();
                    Path path = Paths.get(fullPath);
                    String filename = path.getFileName().toString();

                    filterFileAndCheck(filename, fullPath, "/", mainClasses);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private String getRootPath(String fullPathThisClass) {
        int depth = getDirectoryDepth(fullPathThisClass);
        Path currentPath = Paths.get(fullPathThisClass);

        while (depth > 0) {
            Path parent = currentPath.getParent();
            currentPath = parent;
            depth--;
        }

        return currentPath.toString();
    }

    private void listFilesInDirectory(String directoryPath, List<String> mainClasses) {
        File directory = new File(directoryPath);
        File[] files = directory.listFiles();

        for (File file : files) {
            if (file.isDirectory()) {
                listFilesInDirectory(file.getPath(), mainClasses);
            } else {
                filterFileAndCheck(file.getName(), file.getAbsolutePath(), File.separator, mainClasses);
            }
        }
    }

    public AbstractListModel getListModelWithMainClasses() {
        DefaultListModel listModelMainClasses = new DefaultListModel();
        List<String> mainClasses = getMainClasses();

        for (String clazz : mainClasses) {
            listModelMainClasses.addElement(clazz);
        }

        return listModelMainClasses;
    }

    public void runDemo(String mainClass, String[] args) {
        try {
            jarClassLoader.invokeMain(mainClass, args);
        } catch (Throwable e) {
            e.printStackTrace();
            System.exit(-1);
        }
    }

}
