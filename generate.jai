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
    files := file_list(tprint("%bullet3-3.24/src", #filepath), true);
    for file: files {
        if !ends_with(file, ".h") {
            continue;
        }

        print("%\n", file);
        array_add(*source_files, file);
    }

    array_add(*extra_clang_arguments, "-x", "c++");
    generate_compile_time_struct_checks = false;

    array_add(*system_include_paths, "C:\\Program Files (x86)\\Windows Kits\\10\\Include\\10.0.22621.0\\ucrt");
    array_add(*system_include_paths, "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Tools\\MSVC\\14.31.31103\\include");

    // visitor = my_visitor;

    generate_bindings(opts, "module.jai");
}

// my_visitor :: (decl: *Declaration, parent_decl: *Declaration) -> Declaration_Visit_Result {
//     if !parent_decl {
//         /*if array_find(TYPES_TO_OMIT, decl.name) {
//             decl.decl_flags |= .OMIT_FROM_OUTPUT;
//             return .STOP;
//         }*/

//         if begins_with(decl.name, "VK") || begins_with(decl.name, "Vk") || begins_with(decl.name, "PFN_vk") {
//             decl.decl_flags |= .OMIT_FROM_OUTPUT;
//             return .STOP;
//         }
//     }
//     return .RECURSE;
// }