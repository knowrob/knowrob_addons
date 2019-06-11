
:- module(openease_editor,
    [
        ease_unload_user_package/1,
        ease_load_user_package/1,
        ease_unload_directory/1,
        ease_load_directory/1,
        ease_unload_file/1,
        ease_consult/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- dynamic ease_user_term/2.

is_prolog_source_file(_File) :- true.


ease_unload_user_package(PkgName) :-
  ros_package_path(PkgName,PkgPath),
  atom_concat(PkgPath,'/prolog',PrologDir),
  ease_unload_directory(PrologDir).
ease_load_user_package(PkgName) :-
  ros_package_path(PkgName,PkgPath),
  atom_concat(PkgPath,'/prolog',PrologDir),
  ease_load_directory(PrologDir).


ease_unload_directory(Directory) :-
  directory_files(Directory, Entries),
  forall((
    member(File,Entries),
    atomic_list_concat([
        Directory, '/', File], FilePath),
    exists_file(FilePath),
    is_prolog_source_file(FilePath)),
    ease_unload_file(FilePath)).
ease_load_directory(Directory) :-
  directory_files(Directory, Entries),
  forall((
    member(File,Entries),
    atomic_list_concat([
        Directory, '/', File], FilePath),
    exists_file(FilePath),
    is_prolog_source_file(FilePath)),
    ease_consult(FilePath)).

ease_retract_term(X :- _) :-
  retractall(user:X), !.
ease_retract_term(X) :-
  retractall(user:X).

ease_unload_file(File) :-
  write('Un-Consult file '), writeln(File),
  forall(
    ease_user_term(File,Term),
    ease_retract_term(Term)),
  retractall(ease_user_term(File,_)).

ease_consult(File) :-
  write('Consult file '), writeln(File),
  open(File, read, Fd),
  read(Fd, First),
  read_data(File, First, Fd),
  close(Fd).

read_data(_, end_of_file, _) :- !.
read_data(File, Term, Fd) :-
  expand_term(Term,Expanded),
  assertz(:(user,Expanded)),
  assertz(ease_user_term(File,Expanded)),
  read(Fd, Next),
  read_data(File, Next, Fd).
