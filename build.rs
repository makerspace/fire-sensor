use std::{path::PathBuf, process::Command};

fn main() {
    let output = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .unwrap();
    let git_hash = String::from_utf8(output.stdout).unwrap();
    // println!("cargo:rustc-env=GIT_HASH={}", git_hash);

    let build_id = format!(
        "#{} {}",
        git_hash.trim(),
        chrono::Local::now().format("%Y-%m-%dT%H:%M:%S")
    );

    println!("cargo:rustc-env=BUILD_ID={}", build_id);

    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let mut out_dir2 = out_dir.as_path();
    loop {
        out_dir2 = out_dir2.parent().unwrap();
        if out_dir2.ends_with("build") {
            out_dir2 = out_dir2.parent().unwrap();
            break;
        }
    }
    let out_path = out_dir2.join("build_id");
    std::fs::write(out_path, build_id).unwrap();

    embuild::espidf::sysenv::output();
}
