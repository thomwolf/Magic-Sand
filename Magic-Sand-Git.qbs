import qbs
import qbs.Process
import qbs.File
import qbs.FileInfo
import qbs.TextFile
import "../../../libs/openFrameworksCompiled/project/qtcreator/ofApp.qbs" as ofApp

Project{
    property string of_root: "../../.."

    ofApp {
        name: { return FileInfo.baseName(path) }

        files: [
            'src/main.cpp',
            'src/ofApp.cpp',
            'src/ofApp.h',
            'src\Games\BoidGameController.cpp',
            'src\Games\BoidGameController.h',
            'src\Games\MapGameController.cpp',
            'src\Games\MapGameController.h',
            'src\Games\ReferenceMapHandler.cpp',
            'src\Games\ReferenceMapHandler.h',
            'src\Games\SandboxScoreTracker.cpp',
            'src\Games\SandboxScoreTracker.h',
            'src\Games\vehicle.cpp',
            'src\Games\vehicle.h',
            'src\KinectProjector\KinectGrabber.cpp',
            'src\KinectProjector\KinectGrabber.h',
            'src\KinectProjector\KinectProjector.cpp',
            'src\KinectProjector\KinectProjector.h',
            'src\KinectProjector\KinectProjectorCalibration.cpp',
            'src\KinectProjector\KinectProjectorCalibration.h',
            'src\KinectProjector\TemporalFrameFilter.cpp',
            'src\KinectProjector\TemporalFrameFilter.h',
            'src\KinectProjector\Utils.h',
            'src\KinectProjector\libs\dlib\algs.h',
            'src\KinectProjector\libs\dlib\dassert.h',
            'src\KinectProjector\libs\dlib\enable_if.h',
            'src\KinectProjector\libs\dlib\error.h',
            'src\KinectProjector\libs\dlib\geometry\border_enumerator.h',
            'src\KinectProjector\libs\dlib\geometry\border_enumerator_abstract.h',
            'src\KinectProjector\libs\dlib\geometry\rectangle.h',
            'src\KinectProjector\libs\dlib\geometry\rectangle_abstract.h',
            'src\KinectProjector\libs\dlib\geometry\vector.h',
            'src\KinectProjector\libs\dlib\geometry\vector_abstract.h',
            'src\KinectProjector\libs\dlib\interfaces\cmd_line_parser_option.h',
            'src\KinectProjector\libs\dlib\interfaces\enumerable.h',
            'src\KinectProjector\libs\dlib\interfaces\map_pair.h',
            'src\KinectProjector\libs\dlib\interfaces\remover.h',
            'src\KinectProjector\libs\dlib\is_kind.h',
            'src\KinectProjector\libs\dlib\matrix.h',
            'src\KinectProjector\libs\dlib\matrix\cblas_constants.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\fortran_id.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\gees.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\geev.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\geqrf.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\gesdd.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\gesvd.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\getrf.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\ormqr.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\potrf.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\syev.h',
            'src\KinectProjector\libs\dlib\matrix\lapack\syevr.h',
            'src\KinectProjector\libs\dlib\matrix\matrix.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_assign.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_assign_fwd.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_blas_bindings.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_cholesky.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_conj_trans.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_conv.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_conv_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_data_layout.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_data_layout_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_default_mul.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_eigenvalue.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_exp.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_exp_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_expressions.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_fwd.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_la.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_la_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_lu.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_math_functions.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_math_functions_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_op.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_qr.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_subexp.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_subexp_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_trsm.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_utilities.h',
            'src\KinectProjector\libs\dlib\matrix\matrix_utilities_abstract.h',
            'src\KinectProjector\libs\dlib\matrix\symmetric_matrix_cache.h',
            'src\KinectProjector\libs\dlib\matrix\symmetric_matrix_cache_abstract.h',
            'src\KinectProjector\libs\dlib\memory_manager_stateless\memory_manager_stateless_kernel_1.h',
            'src\KinectProjector\libs\dlib\memory_manager_stateless\memory_manager_stateless_kernel_2.h',
            'src\KinectProjector\libs\dlib\memory_manager_stateless\memory_manager_stateless_kernel_abstract.h',
            'src\KinectProjector\libs\dlib\noncopyable.h',
            'src\KinectProjector\libs\dlib\platform.h',
            'src\KinectProjector\libs\dlib\serialize.h',
            'src\KinectProjector\libs\dlib\stack_trace.h',
            'src\KinectProjector\libs\dlib\uintn.h',
            'src\KinectProjector\libs\dlib\unicode.h',
            'src\KinectProjector\libs\dlib\unicode\unicode.cpp',
            'src\KinectProjector\libs\dlib\unicode\unicode.h',
            'src\KinectProjector\libs\dlib\unicode\unicode_abstract.h',
            'src\KinectProjector\libs\dlib\windows_magic.h',
            'src\SandSurfaceRenderer\ColorMap.cpp',
            'src\SandSurfaceRenderer\ColorMap.h',
            'src\SandSurfaceRenderer\SandSurfaceRenderer.cpp',
            'src\SandSurfaceRenderer\SandSurfaceRenderer.h',
            'src\main.cpp',
            'src\ofApp.cpp',
            'src\ofApp.h',
        ]

        of.addons: [
            'ofxCv',
            'ofxDatGui',
            'ofxKinect',
            'ofxModal',
            'ofxOpenCv',
            'ofxParagraph',
            'ofxXmlSettings',
        ]

        // additional flags for the project. the of module sets some
        // flags by default to add the core libraries, search paths...
        // this flags can be augmented through the following properties:
        of.pkgConfigs: []       // list of additional system pkgs to include
        of.includePaths: []     // include search paths
        of.cFlags: []           // flags passed to the c compiler
        of.cxxFlags: []         // flags passed to the c++ compiler
        of.linkerFlags: []      // flags passed to the linker
        of.defines: []          // defines are passed as -D to the compiler
                                // and can be checked with #ifdef or #if in the code

        // other flags can be set through the cpp module: http://doc.qt.io/qbs/cpp-module.html
        // eg: this will enable ccache when compiling
        //
        // cpp.compilerWrapper: 'ccache'

        Depends{
            name: "cpp"
        }

        // common rules that parse the include search paths, core libraries...
        Depends{
            name: "of"
        }

        // dependency with the OF library
        Depends{
            name: "openFrameworks"
        }
    }

    references: [FileInfo.joinPaths(of_root, "/libs/openFrameworksCompiled/project/qtcreator/openFrameworks.qbs")]
}
