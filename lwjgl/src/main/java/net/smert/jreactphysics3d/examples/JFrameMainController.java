package net.smert.jreactphysics3d.examples;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class JFrameMainController {

    // Directory that contains all the examples
    private final static String EXAMPLE_DIRECTORY = "examples/";
    private final static String ROOT_DIRECTORY_PREFIX = "net/smert/jreactphysics3d/";

    private final JFrameMainModel jFrameMainModel;
    private final JFrameMainView jFrameMainView;
    private final String[] mainArgs;

    public JFrameMainController(String[] args) {
        jFrameMainModel = new JFrameMainModel(EXAMPLE_DIRECTORY, ROOT_DIRECTORY_PREFIX);
        jFrameMainView = new JFrameMainView(this);
        mainArgs = args;
    }

    public void actionRunExample(String exampleClass) {
        jFrameMainModel.runDemo(exampleClass, mainArgs);

    }

    public void actionDefault() {
        jFrameMainView.setListModel(jFrameMainModel.getListModelWithMainClasses());
    }

    public void displayWindow(int x, int y, int width, int height, String title) {
        jFrameMainView.setBounds(x, y, width, height);
        jFrameMainView.setTitle(title);
        jFrameMainView.setVisible(true);
    }

}
