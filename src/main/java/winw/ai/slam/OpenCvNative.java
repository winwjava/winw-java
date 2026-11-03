package winw.ai.slam;

import java.nio.file.*;
import org.opencv.core.Core;

public final class OpenCvNative {
    private OpenCvNative(){}
    public static void load(){
        String path=System.getProperty("slam.opencv.library");
        if(path!=null){System.load(Paths.get(path).toAbsolutePath().toString());return;}
        Path local=Paths.get(System.mapLibraryName(Core.NATIVE_LIBRARY_NAME)).toAbsolutePath();
        if(Files.isRegularFile(local)){System.load(local.toString());return;}
        try{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
        catch(UnsatisfiedLinkError missing){
            // The existing Bytedeco distribution includes the official org.opencv JNI API.
            org.bytedeco.javacpp.Loader.load(org.bytedeco.opencv.opencv_java.class);
        }
    }
}
