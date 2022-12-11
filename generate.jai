#import "Bindings_Generator";
#import "Basic";
#import "File";
#import "String";
#import "BuildCpp";
#import "File_Utilities";

#run {
    using opts: Generate_Bindings_Options;
    array_add(*system_include_paths, GENERATOR_DEFAULT_SYSTEM_INCLUDE_PATH);

    array_add(*libpaths, ".");
    array_add(*libnames, "bullet3");
    array_add(*include_paths, tprint("%libs/bullet", #filepath));
    array_add(*source_files, tprint("%libs/cbullet/cbullet.h", #filepath));
    opts.footer = "#import \"Math\";";

    // array_add(*extra_clang_arguments, "-x", "c++");
    // array_add(*extra_clang_arguments, "-D", "B3_USE_CLEW");
    // array_add(*extra_clang_arguments, "-D", "BT_SHARED");
    generate_compile_time_struct_checks = false;

    visitor :: (decl: *Declaration, parent_decl: *Declaration) -> Declaration_Visit_Result {
        if decl.name == "Vector3" {
            decl.decl_flags |= .OMIT_FROM_OUTPUT;
            return .STOP;
        }
        return .RECURSE;
    }
    opts.visitor = visitor;

    // array_add(*system_include_paths, GENERATOR_DEFAULT_SYSTEM_INCLUDE_PATH);
    array_add(*system_include_paths, "C:\\Program Files (x86)\\Windows Kits\\10\\Include\\10.0.22621.0\\ucrt");
    array_add(*system_include_paths, "C:\\Program Files (x86)\\Windows Kits\\10\\Include\\10.0.22621.0\\um");
    array_add(*system_include_paths, "C:\\Program Files (x86)\\Windows Kits\\10\\Include\\10.0.22621.0\\shared");
    array_add(*system_include_paths, "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Tools\\MSVC\\14.31.31103\\include");

    generate_bindings(opts, "module.jai");

    // TODO need to generate helper methods for each function that has a *Vector3
}
