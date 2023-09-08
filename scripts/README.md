## Procedure to make a ROS release with git submodules

- `scripts/generate_changelog.sh`
- `cd modules && ./git-all pull` (Bring all changelogs to the main repo)
- commit
- `catkin_prepare_release` (it will end with an error "Failed to commit package.xml files:")
- `scripts/post_catkin_prepare_release.sh`