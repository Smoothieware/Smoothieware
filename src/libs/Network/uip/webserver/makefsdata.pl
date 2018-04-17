#!/usr/bin/perl

open(OUTPUT, "> httpd-fsdata2.h");

chdir("httpd-fs");

opendir(DIR, ".");
@files =  grep { !/^\./ && !/(CVS|~)/ } readdir(DIR);
closedir(DIR);

foreach $file (@files) {

    if(-d $file && $file !~ /^\./) {
	print "Processing directory $file\n";
	opendir(DIR, $file);
	@newfiles =  grep { !/^\./ && !/(CVS|~)/ } readdir(DIR);
	closedir(DIR);
	printf "Adding files @newfiles\n";
	@files = (@files, map { $_ = "$file/$_" } @newfiles);
	next;
    }
}

foreach $file (@files) {
    if(-f $file) {

	print "Adding file $file\n";

	open(FILE, $file) || die "Could not open file $file\n";

	$file =~ s-^-/-;
	$fvar = $file;
	$fvar =~ s-/-_-g;
	$fvar =~ s-\.-_-g;
	# for AVR, add PROGMEM here
	print(OUTPUT "static const unsigned char data".$fvar."[] = {\n");
	print(OUTPUT "\t/* $file */\n\t");
	for($j = 0; $j < length($file); $j++) {
	    printf(OUTPUT "%#02x, ", unpack("C", substr($file, $j, 1)));
	}
	printf(OUTPUT "0,\n");


	$i = 0;
	while(read(FILE, $data, 1)) {
	    if($i == 0) {
		print(OUTPUT "\t");
	    }
	    printf(OUTPUT "%#02x, ", unpack("C", $data));
	    $i++;
	    if($i == 10) {
		print(OUTPUT "\n");
		$i = 0;
	    }
	}
	print(OUTPUT "0};\n\n");
	close(FILE);
	push(@fvars, $fvar);
	push(@pfiles, $file);
    }
}

for($i = 0; $i < @fvars; $i++) {
    $file = $pfiles[$i];
    $fvar = $fvars[$i];

    if($i == 0) {
        $prevfile = "NULL";
    } else {
        $prevfile = "file" . $fvars[$i - 1];
    }
    print(OUTPUT "const struct httpd_fsdata_file file".$fvar."[] = {{$prevfile, data$fvar, ");
    print(OUTPUT "data$fvar + ". (length($file) + 1) .", ");
    print(OUTPUT "sizeof(data$fvar) - ". (length($file) + 2) ."}};\n\n"); # don't include the terminating nul
}

print(OUTPUT "#define HTTPD_FS_ROOT file$fvars[$i - 1]\n\n");
print(OUTPUT "#define HTTPD_FS_NUMFILES $i\n");
