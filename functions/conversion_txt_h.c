#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

void convert_txt_to_header(const char *input_file, const char *output_file, const char *variable_name) {
    FILE *infile = fopen(input_file, "r");
    if (!infile) {
        perror("Error opening input file");
        return;
    }

    // First pass: count rows and columns
    int rows = 0;
    int cols = 0;
    int first_row = 1;
    char line[1024];
    
    while (fgets(line, sizeof(line), infile)) {
        if (first_row) {
            char *token = strtok(line, " \t\n");
            while (token) {
                cols++;
                token = strtok(NULL, " \t\n");
            }
            first_row = 0;
        }
        rows++;
    }

    // Rewind file for second pass
    rewind(infile);

    FILE *outfile = fopen(output_file, "w");
    if (!outfile) {
        perror("Error opening output file");
        fclose(infile);
        return;
    }

    // Write header guard
    fprintf(outfile, "#ifndef %s_H\n", variable_name);
    fprintf(outfile, "#define %s_H\n\n", variable_name);

    // Write dimensions
    fprintf(outfile, "#define MAP_ROWS %d\n", rows);
    fprintf(outfile, "#define MAP_COLS %d\n\n", cols);

    // Write array declaration
    fprintf(outfile, "const float %s[MAP_ROWS * MAP_COLS] = {\n", variable_name);

    // Read and write values
    int row = 0;
    while (fgets(line, sizeof(line), infile)) {
        char *token = strtok(line, " \t\n");
        while (token) {
            fprintf(outfile, "    %s.0f%s", token, 
                    (row == rows-1 && token == NULL) ? "" : ",");
            token = strtok(NULL, " \t\n");
        }
        fprintf(outfile, "\n");
        row++;
    }

    fprintf(outfile, "};\n\n");
    fprintf(outfile, "#endif // %s_H\n", variable_name);

    fclose(infile);
    fclose(outfile);

    printf("Header file '%s' generated successfully with %dx%d map.\n", 
           output_file, rows, cols);
}

int main() {
    const char *input_file = "./Mapas/MAP_2_50_50.txt";
    const char *output_file = "map.h";
    const char *variable_name = "MAP_DATA";

    convert_txt_to_header(input_file, output_file, variable_name);
    return 0;
}