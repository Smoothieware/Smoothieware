In order for a Pull request to be accepted it must meet certain standards and guidelines.

In addition to following the [Coding-Standards](http://smoothieware.org/coding-standards) it is suggested the following be adopted.

* There must be a detailed description of the change, why it was added and what the use case is, and a link to the documentation to use it in the wiki if applicable.
* Only one feature per pull request, if you change multiple files with unrelated changes it will be rejected.
* Do not make gratuitous changes. If your coding style is different from the one adopted by most of the Smoothie codebase, do not make changes to suit your style. Peoples styles differ, and the new code you write may reflect your coding style, that is ok, but changing existing code to match your style is not ok. Doing so will delay the acceptance of your code, or have it rejected entirely.
* Maintain backwards compatibility. Do not make changes that will break other peoples systems, it is not acceptable to force everyone to change their config files to accommodate your change. If necessary add new configs to enable your change if it is breaking backward compatibility.
* Do not make changes that are specific to your setup. Smoothie needs to work in many different environments.
* Do not make changes that customize messages specific to your company or implementation. You are welcome to do this in your own fork and binary, it is not ok to try to merge that into the upstream repository.
* Test your changes is as many different configurations as you can, eg different arm solutions. In the Pull request state what tests have been done and what tests need to be done.
* Do not commit changes to the FirmwareBin directory or any binary files in the repository.
* Do not reformat the Config Samples to suit your style.
* Add examples of your config entries to the ConfigSamples/Snippets folder, only change existing config samples if absolutely necessary
* If you must correct existing comments (like correct spelling mistakes or typos) then submit them as separate pull requests
* Make sure your Pull request is against the current edge branch, and passes the Travis CI.
* All comments and code must be in English, this includes names of functions.

It may seem like a pain to follow these rules, but it takes the maintainers valuable time and effort to review and test pull requests, these guidelines help reduce that effort.

