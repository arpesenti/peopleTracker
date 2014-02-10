/** Credit goes to 
http://stackoverflow.com/questions/3580752/java-dynamically-loading-a-class?lq=1
*/
package edu.ucsd;

import java.lang.reflect.Method;
import java.net.URL;
import java.net.URLClassLoader;
import java.io.IOException;
import java.io.File;

public class ClassPathHacker{

    private static final Class<URLClassLoader> URLCLASSLOADER =
        URLClassLoader.class;
    private static final Class<?>[] PARAMS = new Class[] { URL.class };

    public static void addFile(final String s) throws IOException{
        addFile(new File(s));
    }

    public static void addFile(final File f) throws IOException{
        addURL(f.toURI().toURL());
    }

    public static void addURL(final URL u) throws IOException{

        final URLClassLoader urlClassLoader = getUrlClassLoader();

        try{
            final Method method = getAddUrlMethod();
            method.setAccessible(true);
            method.invoke(urlClassLoader, new Object[] { u });
        } catch(final Exception e){
            throw new IOException(
                "Error, could not add URL to system classloader");
        }

    }

    private static Method getAddUrlMethod()
        throws NoSuchMethodException{
        if(addUrlMethod == null){
            addUrlMethod =
                URLCLASSLOADER.getDeclaredMethod("addURL", PARAMS);
        }
        return addUrlMethod;
    }

    private static URLClassLoader urlClassLoader;
    private static Method addUrlMethod;

    private static URLClassLoader getUrlClassLoader(){
        if(urlClassLoader == null){
            final ClassLoader sysloader = 
                ClassLoader.getSystemClassLoader();
            if(sysloader instanceof URLClassLoader){
                urlClassLoader = (URLClassLoader) sysloader;
            } else{
                throw new IllegalStateException(
                    "Not an UrlClassLoader: "
                    + sysloader);
            }
        }
        return urlClassLoader;
    }

}
