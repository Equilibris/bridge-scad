{
  description = "Flake utils demo";

  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = nixpkgs.legacyPackages.${system}; in
      {
        devShell = pkgs.mkShell {
          packages = with pkgs; [
            python310

            texlab
            (texlive.combine {
              inherit (texlive)
                scheme-medium
                runcode morewrites tcolorbox environ apacite
                xifthen ifmtarg framed paralist
                titlesec
                blindtext;
            })
          ];
        };
      }
    );
}
