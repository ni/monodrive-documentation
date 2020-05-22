def fix_files(readme_list):
    for file in readme_list:
        os.rename(file, "temp_" + file):
        with open (file, 'w') as fout, open("temp_" + file, "r") as fin:
            for line in fin.readlines():
                if "<img" in line:
                    image_name = line.split('/')[-1]
                    new_line = '/'.join(line.split('/')[:-1]) + '/'+ file.split('.md')[0]+".png"+"\"" + " "
                    print(new_line)
                    fout.write(new_line)
                    
                else:
                    fout.write(line)