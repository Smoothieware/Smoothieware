## How to file an issue

If you encounter an issue with SmoothieWare, you are welcome to
[submit an issue](https://github.com/Smoothieware/Smoothieware/issues)

**YOU MUST** provide the complete output from the version command or M115, the config and the output from M503, otherwise the issue will be summarily closed.

**Please note** while other projects will use github issues as a general discussion forum, we try to restrict the issues to **actual** issues only. If you want to discuss new features, ask questions, or anything else that is not an issue, please go to the Smoothie forums, mailing lists, IRC, social media etc.

**DO NOT ASK QUESTIONS HERE** they will not be answered and the issue will be summarily closed.

Before you do that for the first time though please take a moment to read the
following section *completely*. Thank you! :)

### What should I do before submitting an issue?

1. **this is not the right issue tracker if you are running a
   forked version of Smoothieware**. Seek help for such unofficial versions from
   the people maintaining them instead.

2. Please make sure to **test out the current edge version** of Smoothieware to see
   whether the problem you are encountering still exists, and **test with a clean config**.

3. The problem still exists? Then please look through the
   [existing tickets](https://github.com/Smoothieware/Smoothieware/issues?state=open)
   to check if there already exists a report of the issue you are encountering.

   **Very important:** Please make absolutely sure that if you find a bug that looks like
   it is the same as your's, it actually behaves the same as your's. E.g. if someone gives steps
   to reproduce his bug that looks like your's, reproduce the bug like that if possible,
   and only add a "me too" if you actually can reproduce the same
   issue. Also **provide all information** as [described below](#what-should-i-include-in-a-bug-report)
   and whatever was additionally requested over the course of the ticket
   even if you "only" add to an existing ticket. The more information available regarding a bug, the higher
   the chances of reproducing and solving it. But "me too" on an actually unrelated ticket
   makes it more difficult due to on top of having to figure out the original problem
   there's now also a [red herring](https://en.wikipedia.org/wiki/Red_herring) interfering - so please be
   very diligent here!

### What should I include in a bug report?

Always use the following template (you can remove what's within `[...]`, that's
only provided here as some additional information for you), **even if only adding a
"me too" to an existing ticket**:

#### What were you doing?

    [Please be as specific as possible here. The maintainers will need to reproduce
    your issue in order to fix it and that is not possible if they don't know
    what you did to get it to happen in the first place. If you encountered
    a problem with specific files of any sorts, make sure to also include a link to a file
    with which to reproduce the problem.]

#### What did you expect to happen?

#### What happened instead?

#### Branch & Commit or Version of Smoothieware

    [Can be found with the version command. (@version in pronterface)]

#### Hardware model you are using (Make, and version)

    [eg Smoothieboard 5X V1b, Azteeg X5 Mini V2, ... etc]

#### Host and Operating System running the Host

    [eg Pronterface, Octoprint, linux x64]

#### Link to and console logs

    [On gist.github.com or pastebin.com. Always include and never truncate.]
    
#### Link to running config and M503 results

    [On gist.github.com or pastebin.com. Always include and never truncate.]

I have read the [Wiki](http://smoothieware.org) and Especially the [Trouble Shooting section](http://smoothieware.org/troubleshooting)... 

Copy-paste this template **completely**. Do not skip any lines!

It might happen that you are asked to provide a more thorough crash log which you can find out how to do [here](http://smoothieware.org/mri-debugging
)
