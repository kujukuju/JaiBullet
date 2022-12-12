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
    opts.header = "size_t :: u64;";
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
    create_non_pointer_methods("module.jai");
}

create_non_pointer_methods :: (filename: string) {
    contents, success := read_entire_file(filename);
    if !success {
        return;
    }

    lines: [..] string;
    array_copy(*lines, split(contents, "\n"));

    for i: 0..lines.count - 1 {
        if contains(lines[i], "*Vector3") || contains(lines[i], "*[4] Vector3") || contains(lines[i], "*float") {
            params_start := index_of_string_nocase(lines[i], "(");
            params_end := index_of_string_nocase(lines[i], ")");
            pre_string := slice(lines[i], 0, params_start + 1);
            if contains(pre_string, "#type") {
                continue;
            }

            params_string := slice(lines[i], params_start + 1, params_end - params_start - 1);
            post_string := slice(lines[i], params_end, lines[i].count - params_end);

            function_name := split(pre_string, " :: ")[0];
            params := split(params_string, ",");
            returns_value := contains(post_string, "->");

            new_line: String_Builder;
            append(*new_line, pre_string);

            call_line: String_Builder;
            append(*call_line, "\t");
            if returns_value {
                append(*call_line, "return ");
            }
            append(*call_line, function_name);
            append(*call_line, "(");

            for a: 0..params.count - 1 {
                param_parts := split(params[a], ":");

                append(*new_line, trim(param_parts[0]));
                append(*new_line, ": ");

                if contains(param_parts[1], "*Vector3") {
                    append(*new_line, "Vector3");

                    append(*call_line, "*");
                    append(*call_line, trim(param_parts[0]));
                } else if contains(param_parts[1], "*[4] Vector3") {
                    append(*new_line, "[4] Vector3");

                    append(*call_line, "*");
                    append(*call_line, trim(param_parts[0]));
                } else if contains(param_parts[1], "*float") {
                    append(*new_line, "float");

                    append(*call_line, "*");
                    append(*call_line, trim(param_parts[0]));
                } else {
                    append(*new_line, trim(param_parts[1]));
                    append(*call_line, trim(param_parts[0]));
                }

                if a < params.count - 1 {
                    append(*new_line, ", ");
                    append(*call_line, ", ");
                }
            }

            corrected_post_string := replace(post_string, "#foreign bullet3;", "{");
            // corrected_post_string = replace(corrected_post_string, "#c_call;", "{");
            append(*new_line, corrected_post_string);
            append(*call_line, ");");

            array_insert_at(*lines, builder_to_string(*new_line), i + 1);
            array_insert_at(*lines, builder_to_string(*call_line), i + 2);
            array_insert_at(*lines, "}\n", i + 3);

            i += 3;
        }
    }

    new_file := join(..lines, "\n");
    write_entire_file("module.jai", new_file);
}
