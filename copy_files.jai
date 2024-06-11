
#run {
    file_delete("bullet3.dll");
    file_delete("bullet3.lib");
    copy_file("libs/cmake-build-release/bin/bullet3.dll", "bullet3.dll");
    copy_file("libs/cmake-build-release/bin/bullet3.lib", "bullet3.lib");

    file_delete("../FatGoblins/bullet3.dll");
    file_delete("../FatGoblins/bullet3.lib");
    copy_file("libs/cmake-build-release/bin/bullet3.dll", "../FatGoblins/bullet3.dll");
    copy_file("libs/cmake-build-release/bin/bullet3.lib", "../FatGoblins/bullet3.lib");

    file_delete("../FatGoblinsServer/bullet3.dll");
    file_delete("../FatGoblinsServer/bullet3.lib");
    copy_file("libs/cmake-build-release/bin/bullet3.dll", "../FatGoblinsServer/bullet3.dll");
    copy_file("libs/cmake-build-release/bin/bullet3.lib", "../FatGoblinsServer/bullet3.lib");
};

#import "File";
#import "File_Utilities";